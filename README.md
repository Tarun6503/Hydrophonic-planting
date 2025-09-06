# Hydrophonic-planting

The project begins with the initialization stage, where the ESP32 (or Arduino) controller powers on and all sensors such as pH, EC, water temperature, and air temperature/humidity are activated. The Real-Time Clock (RTC) also sets the correct time, which will be used for scheduling the grow lights and daily reset functions.

Once the system is running, the controller continuously monitors the environment. The sensors collect real-time data on the acidity (pH), nutrient concentration (EC), water temperature, and air conditions. These readings are compared with the predefined target setpoints that are suitable for plant growth.

When deviations are detected, the system automatically adjusts the solution through dosing. If the pH level is too high, the controller activates the pH Down pump for a short duration. If it is too low, the pH Up pump is used. Similarly, if the nutrient concentration is below the target, Nutrient A and Nutrient B pumps are alternately activated to maintain balance. The dosing is done in small pulses with mixing delays in between, which prevents sudden overdosing that could harm the plants.

Apart from nutrient control, the project also manages the plant’s environment. The grow lights are turned on and off according to a set daily schedule, mimicking natural day and night cycles. The fan is activated whenever the air temperature or humidity rises above a threshold, ensuring proper ventilation and cooling. The main water pump operates either continuously or in cycles, depending on the type of hydroponic system used, to keep the nutrient solution circulating and oxygenated.

For safety, the system resets daily dosing counters at midnight to avoid excessive corrections. In case sensors provide faulty or unstable readings, the controller disables automatic dosing and locks the pumps to protect the plants. This ensures that the system does not harm the crops due to incorrect measurements.

Through this fully automated process, the hydroponics planting project provides plants with a stable and controlled growing environment. It reduces manual intervention, optimizes water and nutrient use, and enhances plant growth and yield by maintaining ideal conditions around the clock.
