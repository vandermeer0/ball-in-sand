import matplotlib.pyplot as plt

with open('total_kek12.txt', 'r') as file:
    line = file.readline()
    numbers = [float(num) for num in line.split()]

energy = []
height = []
time = []

# Используйте переменную i для итерации по времени
for i in range(500):
    energy.append(numbers[3 * i])
    height.append(numbers[3 * i + 1])
    time.append(i * 1 / 125)  # Исправлено на корректный расчет времени

# График Кинетической энергии от времени
plt.figure(figsize=(12, 4))

plt.subplot(131)
plt.plot(time, energy, label='Кин. энергия')
plt.title('График кинетической энергии от времени')
plt.xlabel('Время, сек')
plt.ylabel('Кинетическая энергия, Дж')
plt.legend()

# График Высоты от времени
plt.subplot(132)
plt.plot(time, height, label='Высота')
plt.title('График высоты от времени')
plt.xlabel('Время, сек')
plt.ylabel('Высота, м')
plt.legend()

# График Кинетической энергии от высоты
plt.subplot(133)
plt.plot(height, energy, label='Кинетическая энергия')
plt.title('График кинетической энергии от высоты')
plt.xlabel('Высота, м')
plt.ylabel('Кинетическая Энергия, Дж')
plt.legend()

plt.tight_layout()  # Добавлено для предотвращения перекрытия осей
plt.show()
