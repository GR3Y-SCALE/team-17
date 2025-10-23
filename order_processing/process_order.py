import csv, os

class Process_Order:
    def import_order(self, debug=False):
        '''
        Reads order from CSV and returns list of order details, Bottle and Weetbots ignored and list resorted by order num
        '''
        item_num = []
        picking_station_num = []
        shelf_num = []
        bay_num = []
        bay_height = []
        item_name = []
        if debug: print(os.path.abspath(os.getcwd()))
        with open("order_processing/Order_2_2025.csv", mode="r", encoding='utf-8-sig') as csv_file:
            csv_reader = csv.reader(csv_file)

            for row in csv_reader:
                # print(row)
                if row[5] == 'Bottle' or row[5] == 'Weetbots' or row[5] == 'Item Name':
                    continue
                else:
                    item_num.append(int(row[0]))
                    picking_station_num.append(int(row[1]))
                    shelf_num.append(int(row[2]))
                    bay_num.append(int(row[3]))
                    bay_height.append(int(row[4]))
                    item_name.append(str(row[5]))
        print("[ OK ] Order processed.")
        if debug:
            print(item_num)
            print(picking_station_num)
            print(shelf_num)
            print(bay_num)
            print(bay_height)
            print(item_name)
        return item_num, picking_station_num, shelf_num, bay_num, bay_height, item_name
    

def main():
    po = Process_Order()
    po.import_order(debug=True)

if __name__ == '__main__':
    main()