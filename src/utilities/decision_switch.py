class DecisionBlock():

    def __init__(self, control_list):

        self.control_names = control_list
        self.control_dict = {control_list[i]: i for i in range(len(control_list))}
        print(self.control_dict)


if __name__ == '__main__':

    cmd_list   = ["locate_block"]
    db         = DecisionBlock(cmd_list)