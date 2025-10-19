import csv


with open("Order_1_2025.csv", mode="r", encoding='utf-8-sig') as csv_file:
    csv_reader = csv.reader(csv_file)

    for row in csv_reader:
        print(row)