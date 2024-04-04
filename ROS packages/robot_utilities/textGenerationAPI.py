#!/usr/bin/env python3
import nlpcloud
import os

API_KEY = os.environ.get('MY_API_KEY')

def generate_text(prompt, model="finetuned-llama-2-70b"):
    client = nlpcloud.Client(model, API_KEY, gpu=True)
    res = client.generation(prompt, max_length=2048, remove_input = True)
    return res["generated_text"]
