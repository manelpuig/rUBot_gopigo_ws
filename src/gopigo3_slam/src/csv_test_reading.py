import csv
fichero = 'waypoints3.csv'
results = []
with open(fichero) as csvfile:
    reader = csv.reader(csvfile, delimiter=";") 
    for lista in reader: # Cada fila es una lista
        results.append(lista)
print(results [1][1])
print(results [1])