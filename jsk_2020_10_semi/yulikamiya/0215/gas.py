import pprint
import json
import requests
import time

def main():
    body = {'name': 'YuliKamiya', 'temp': 'green', 'time': int(time.time())}
    response = requests.post(
        'https://script.google.com/macros/s/AKfycby3MbYwphpz9T3xj3V5huRFXaZwOOtdViEk6fof6erfHwTGWgY02rEc/exec',
        json.dumps(body),
        headers={'Content-Type': 'application/json', 'Content-Length':'200'})
    pprint.pprint(response)


if __name__=='__main__':
    main()
