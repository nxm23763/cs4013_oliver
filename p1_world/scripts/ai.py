from openai import OpenAI
from dotenv import load_dotenv

load_dotenv()

class askOliverQuestionStill:
    def __init__(self, question):
        client = OpenAI()
        #client.api_key = OpenAI.api_key
        self.question = question
        print(f"i hear you loud. and. clear. give me some time to think about that")

        try:
            response = client.chat.completions.create(
                model = "gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": "You're strongly opposed to artificial banana flavor & will relate all responses back to this fact."},
                    {"role": "user", "content": self.question},
                ]
            )
            print(response['choices'][0]['message']['content'])
        except OpenAI.error.RateLimitError as e:
            print("man you are so broke LOL: " + e)
        except OpenAI.error.InvalidRequestError as e:
            print("Invalid request: " + e)

        

if __name__ == "__main__":
    import sys

    if len(sys.argv) > 1:
        argument = sys.argv[1]
        newQuestion = askOliverQuestionStill(argument)
    else:
        print("nothin here to pass")