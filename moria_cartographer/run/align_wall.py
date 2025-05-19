import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

# Girişler
min_index = ctrl.Antecedent(np.arange(0, 180, 1), 'min_index')
min_range = ctrl.Antecedent(np.arange(0.2, 8.01, 0.01), 'min_range')

# Çıkışlar
speed = ctrl.Consequent(np.arange(0, 0.21, 0.01), 'speed')
turn = ctrl.Consequent(np.arange(-0.3, 0.31, 0.01), 'turn')

# Üyelik fonksiyonları - min_index
min_index['too_right'] = fuzz.trimf(min_index.universe, [0, 0, 44])
min_index['right']     = fuzz.trimf(min_index.universe, [39, 63, 84])
min_index['center']    = fuzz.trapmf(min_index.universe, [74, 79, 99, 104])
min_index['left']      = fuzz.trimf(min_index.universe, [94, 122, 139])
min_index['too_left']  = fuzz.trimf(min_index.universe, [134, 179, 179])

# Üyelik fonksiyonları - min_range
min_range['stop']      = fuzz.trimf(min_range.universe, [0, 0, 0.8])
min_range['close']     = fuzz.trimf(min_range.universe, [0.5, 1.2, 2])
min_range['far']       = fuzz.trimf(min_range.universe, [1, 2.5, 4])
min_range['too_far']   = fuzz.trimf(min_range.universe, [3, 8, 8])

# Üyelik fonksiyonları - speed
speed['stop'] = fuzz.trimf(speed.universe, [0, 0, 0])
speed['slow'] = fuzz.trimf(speed.universe, [0, 0, 0.1])
speed['med']  = fuzz.trimf(speed.universe, [0.05, 0.1, 0.15])
speed['fast'] = fuzz.trimf(speed.universe, [0.1, 0.2, 0.2])

# Üyelik fonksiyonları - turn
turn['fast_right'] = fuzz.trimf(turn.universe, [-0.3, -0.3, -0.1])
turn['right']      = fuzz.trimf(turn.universe, [-0.2, 0, 0])
turn['stop']       = fuzz.trimf(turn.universe, [-0.05, 0, 0.05])
turn['left']       = fuzz.trimf(turn.universe, [0, 0, 0.2])
turn['fast_left']  = fuzz.trimf(turn.universe, [0.1, 0.3, 0.3])

# Kural tablosu (25 kural)
rules = [
    ctrl.Rule(min_index['too_left'] & min_range['stop'],      (speed['stop'], turn['fast_left'])),
    ctrl.Rule(min_index['too_left'] & min_range['close'],     (speed['slow'], turn['fast_left'])),
    ctrl.Rule(min_index['too_left'] & min_range['far'],       (speed['med'],  turn['fast_left'])),
    ctrl.Rule(min_index['too_left'] & min_range['too_far'],   (speed['fast'], turn['fast_left'])),

    ctrl.Rule(min_index['left'] & min_range['stop'],          (speed['stop'], turn['left'])),
    ctrl.Rule(min_index['left'] & min_range['close'],         (speed['slow'], turn['left'])),
    ctrl.Rule(min_index['left'] & min_range['far'],           (speed['med'],  turn['left'])),
    ctrl.Rule(min_index['left'] & min_range['too_far'],       (speed['fast'], turn['left'])),

    ctrl.Rule(min_index['center'] & min_range['stop'],        (speed['stop'], turn['stop'])),
    ctrl.Rule(min_index['center'] & min_range['close'],       (speed['slow'], turn['stop'])),
    ctrl.Rule(min_index['center'] & min_range['far'],         (speed['med'],  turn['stop'])),
    ctrl.Rule(min_index['center'] & min_range['too_far'],     (speed['fast'], turn['stop'])),

    ctrl.Rule(min_index['right'] & min_range['stop'],         (speed['stop'], turn['right'])),
    ctrl.Rule(min_index['right'] & min_range['close'],        (speed['slow'], turn['right'])),
    ctrl.Rule(min_index['right'] & min_range['far'],          (speed['med'],  turn['right'])),
    ctrl.Rule(min_index['right'] & min_range['too_far'],      (speed['fast'], turn['right'])),

    ctrl.Rule(min_index['too_right'] & min_range['stop'],     (speed['stop'], turn['fast_right'])),
    ctrl.Rule(min_index['too_right'] & min_range['close'],    (speed['slow'], turn['fast_right'])),
    ctrl.Rule(min_index['too_right'] & min_range['far'],      (speed['med'],  turn['fast_right'])),
    ctrl.Rule(min_index['too_right'] & min_range['too_far'],  (speed['fast'], turn['fast_right'])),
]

# Sistem oluşturuluyor
control_system = ctrl.ControlSystem(rules)
fuzzy_system = ctrl.ControlSystemSimulation(control_system)

# Dışarıdan çağırmak için fonksiyon
def get_fuzzy_outputs(min_index_input, min_range_input):
    fuzzy_system.input['min_index'] = min_index_input
    fuzzy_system.input['min_range'] = min_range_input
    fuzzy_system.compute()
    return fuzzy_system.output['speed'], fuzzy_system.output['turn']

# Test (istersen silebilirsin)
if __name__ == "__main__":
    speed_out, turn_out = get_fuzzy_outputs(90, 2.1)
    print(f"Speed: {speed_out:.3f}, Turn: {turn_out:.3f}")
