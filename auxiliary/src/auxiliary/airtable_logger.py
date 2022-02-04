import os
import json
from pyairtable import Table

class AirtableLogger():
    def __init__(self, config_file_json=None):
        self.api_key = os.environ['AIRTABLE_API_KEY']
        self.row = None
        self.base_id = None
        self.table_id = None
        self.column_names = None
        self.row_dict = {}

        if config_file_json:
            self.init_from_json(config_file_json)

        self.table = Table(self.api_key, self.base_id, self.table_id)

    def __str__(self):
        out = "=====================\n"
        for key in self.row_dict:
            out = out+"%s : %s\n" % (key, self.row_dict[key])
        out = out + "====================="
        return out

    def init_from_json(self, config_file_json):
        json_object = json.load(open(config_file_json))
        self.base_id = str(json_object['base_id'])
        self.table_id = str(json_object['table_id'])
        self.columns = dict(json_object['columns'])
        for key in self.columns:
            if self.columns[key]['type'] == 'str':
                self.row_dict.update({self.columns[key]['name']: 'default'})
            elif self.columns[key]['type'] == 'float':
                self.row_dict.update({self.columns[key]['name']: 0.0})
            else:
                print("%s has unsupported datatype" % self.columns[key]['name'])

    def get_field(self, field):
        try:
            return self.row_dict[field]
        except Exception as e:
            print(e)

    def set_field(self, field, value):
        if type(self.row_dict[field]) == type(value):
            self.row_dict[field] = value
            return True
        else:
            print('provided value datatype is different from field')
            return False

    def add_to_field(self, field, value):
        if type(self.row_dict[field]) == type(value):
            self.row_dict[field] += value
            return True
        else:
            print('provided value datatype is different from field')
            return False

    def reset_data(self):
        for key in self.columns:
            if self.columns[key]['type'] == 'str':
                self.row_dict[self.columns[key]['name']] = 'default'
            elif self.columns[key]['type'] == 'float':
                self.row_dict[self.columns[key]['name']] = 0.0
            else:
                pass

    def log(self):
        try:
            self.table.create(self.row_dict)
            print("logged %s to Airtable" % self.row_dict['TimeStamp'])
        except Exception as e:
            print(e)


if __name__ == '__main__':
    table = AirtableLogger(config_file_json='/home/rupeek/armws/src/remoteAssayingMachine/interbotixarm/config/vault_airtable.json')
    # print(table)
    # print(table.add_to_field('js_1', 10))
    # print(table.set_field('TimeStamp', 'date time'))
    # table.log()
    table.add_to_field('total_time', 45.5)
    table.add_to_field('total_time', 3.5)
    print(table.get_field('total_time'))

    
    