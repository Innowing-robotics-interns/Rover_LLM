import torch
import re
from transformers import AutoModelForCausalLM, AutoTokenizer, BitsAndBytesConfig
from peft import PeftModel
from utils import get_path, load_prompt


class LMP:
    def __init__(self, name, cfg, fixed_vars={}, variable_vars={}):
        self.name = name
        self.cfg = cfg
        self.model = None
        self.tokenizer = None
        self.fixed_vars = fixed_vars
        self.variable_vars = variable_vars
        self.load_model()
    
    
    def config(self):
        self.model_id = get_path(self.cfg['model'])
        self.adapter_id = get_path(self.cfg['adapter'])
        self.sys_prompts = load_prompt(self.cfg['sys_prompts'])
        self.user_prompts = load_prompt(self.cfg['user_prompts'])
    
    
    def format_chat_template(self, prompt, document=None):
        if (self.variable_vars is None):
            custom_import = ''
        else:
            custom_import = f"from LLM_lib import {', '.join(self.variable_vars.keys())}"
        self.user_prompts = self.user_prompts.replace('{custom_import}', custom_import)
        
        user1 = f"I would like you to help me write Python code to control a robot navigation operating in indoor environment. Please complete the code every time when I give you new query. Pay attention to appeared patterns in the given context code. Be thorough and thoughtful in your code. Do not include any import statement. Do not repeat my question. Do not provide any text explanation (comment in code is okay). I will first give you the context of the code below:\n\n```\n{self.user_prompts}\n```\n\nNote that x is back to front, y is left to right, and z is bottom to up."
        assistant1 = f'Got it. I will complete what you give me next.'
        # prompt = "Command: " + prompt
        prompt = f'Query: {prompt}'
        messages = [
            {"role": "system", "content": self.sys_prompts},
            {"role": "user",   "content": user1},
            {"role": "assistant", "content": assistant1},
            {"role": "user",   "content": prompt+'\n'},
        ]

        input_ids = self.tokenizer.apply_chat_template(
            messages,
            add_generation_prompt=True,
            documents=document,
            return_tensors="pt"
        ).to("cuda")
    
        return input_ids
    
    
    def generate(self, input_ids):
        self.model.eval()
        with torch.no_grad():
            result = self.tokenizer.decode(self.model.generate(inputs=input_ids, max_new_tokens=self.cfg['max_new_tokens'], pad_token_id=self.tokenizer.eos_token_id)[0], skip_special_tokens=True)
            # result = self.tokenizer.decode(self.model.generate(**input_ids, max_new_tokens=1000, pad_token_id=self.tokenizer.eos_token_id)[0], skip_special_tokens=True)
        
        # Define regular expression pattern to extract the behavior tree from the complete output
        # pattern = r'assistant(.*?)# done' # depends on the prompt
        pattern = r'assistant\n(.*)'
        matches = re.findall(pattern, result, re.DOTALL)
        # print("="*80)
        # print(result)
        # print("="*80)
        # print(matches[-1])
        # print("="*80)
        final = re.findall(r'assistant(.*)', matches[-1], re.DOTALL)[-1] + "# done"
        print(final)
        print(type(final))
        print("="*80)
        if not final:
            return None
        # execute the code
        gvars = self.fixed_vars | self.variable_vars
        lvars = {}
        success = safe_to_run(final,gvars, lvars)
        # if success: #and self.cfg['save_output']:
        #     print(lvars['result'])
        
        return final, success
    
    
    def load_model(self):
        # config
        self.config()
        quantization_config = BitsAndBytesConfig(load_in_4bit=True) if self.cfg['load_in_4bit'] \
                              else BitsAndBytesConfig(load_in_8bit=True)
        token = "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"

        # Load base model
        base_model = AutoModelForCausalLM.from_pretrained(
            pretrained_model_name_or_path = self.model_id,
            quantization_config = quantization_config,
            torch_dtype = torch.float16,
            device_map = {"":0}, #auto
            # trust_remote_code = True,
            token=token,
            local_files_only=True, # local LLM
        )
        # Load tokenizer
        self.tokenizer = AutoTokenizer.from_pretrained(
            pretrained_model_name_or_path = self.model_id, 
            token=token,
            local_files_only=True
        )
        
        if self.adapter_id is None:
            self.model = base_model
            return
        # Load fine-tuned model
        finetuned_model = PeftModel.from_pretrained(base_model, self.adapter_id, device_map = {"":0},) #auto)
        # finetuned_model = finetuned_model.merge_and_unload()

        device = torch.device("cuda")# if torch.cuda.is_available() else "cpu")
        finetuned_model = finetuned_model.to(device)
        self.model = finetuned_model
        return
    
    def __call__(self, prompt):
        input_text = self.format_chat_template(prompt)
        return self.generate(input_text)
    
# execute module
def safe_to_run(code, gvars=None, lvars=None):
    forbidden = ['import','__','exec','eval']
    for word in forbidden:
        assert word not in code, f'forbidden word "{word}" in code'
    if gvars is None:
        gvars = {}
    if lvars is None:
        lvars = {}
    try:
        exec(code, gvars, lvars)
        return True
    except Exception as e:
        print(f'Error codes: {e}')
        return False
        
        