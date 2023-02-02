import json


class frontEndTest(object):
    def __init__(self):
        self.primitives_to_execute = []

    def add_to_list(self, command):
        self.primitives_to_execute.append(command)

    def print_list(self):
        print(self.primitives_to_execute)

    def get_queue_list(self):
        # json_string = json.dumps(self.primitives_to_execute)
        # print(json_string)
        # return json_string
        return self.primitives_to_execute

