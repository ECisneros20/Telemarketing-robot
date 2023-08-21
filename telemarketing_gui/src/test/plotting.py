import pandas as pd
import matplotlib.pyplot as plt

number_test_for_preset = 10

df = pd.read_csv("./telemarketing_gui/src/test/metrics_test_bark.csv")
x_axis = [str(i) for i in range(number_test_for_preset)]
x_axis.append("Avg")

# Initialise the subplot function using number of rows and columns
figure, axes = plt.subplots(2, 5, figsize = (15, 12))

for i in range(10):

    ax = axes[int(i/5), i%5]
    ax.bar(x_axis, df[str(i)])
    ax.set_xlabel("Tests")
    ax.set_ylabel("Time spent (s)")
    ax.set_title(f"Test for the preset #{i}")

# Combine all the operations and display
plt.show()



for bar in plots.patches:
   
  # Using Matplotlib's annotate function and
  # passing the coordinates where the annotation shall be done
  # x-coordinate: bar.get_x() + bar.get_width() / 2
  # y-coordinate: bar.get_height()
  # free space to be left to make graph pleasing: (0, 8)
  # ha and va stand for the horizontal and vertical alignment
    plots.annotate(format(bar.get_height(), '.2f'),
                   (bar.get_x() + bar.get_width() / 2,
                    bar.get_height()), ha='center', va='center',
                   size=15, xytext=(0, 8),
                   textcoords='offset points')