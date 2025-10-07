# ODrive S1 Configuration & Calibration Guide

## 📋 **Hardware Setup**

This script is designed for the following configuration:
- **Controller**: ODrive S1
- **Motor**: BLDC motor with integrated **Hall effect sensors** (3-wire hall)
- **Encoder**: **Absolute encoder** (SPI) mounted on **gear-reduced output shaft**
- **Gear reduction**: Between motor shaft and load (e.g., 10:1 planetary gearbox)

### **Sensor Mapping Strategy**
```
Motor Shaft (High Speed)
    ↓
Hall Sensors → Commutation Feedback (coarse, 60° resolution)
    ↓
Gearbox (e.g., 10:1 reduction)
    ↓
Output Shaft (Low Speed, High Torque)
    ↓
Absolute Encoder → Position Feedback (precise, 16384 CPR)
```

---

## 🔧 **Before You Start - Parameter Checklist**

### **1. Measure/Find Motor Parameters**

Open the script and update these values:

#### **Motor Specifications** (lines 35-44)
```python
POLE_PAIRS = 7              # Count magnets on rotor, divide by 2
MOTOR_KV = 170              # From motor datasheet (RPM/V)
MOTOR_RESISTANCE = 0.5      # Measure with multimeter (phase-to-phase / 2)
MOTOR_INDUCTANCE = 0.0005   # Typically 0.0001-0.001 H (let ODrive measure)
```

**How to find pole pairs:**
- Open motor and count magnets on rotor
- Pole pairs = magnets / 2
- Example: 14 magnets = 7 pole pairs

**How to measure resistance:**
```bash
# Disconnect motor from ODrive
# Measure resistance between any two motor phases
# Divide by 2 to get per-phase resistance
```

#### **Current Limits** (lines 47-51)
```python
CALIBRATION_CURRENT = 5.0   # Safe moderate value for calibration
CURRENT_LIMIT = 20.0        # Check motor datasheet for continuous rating!
CURRENT_SOFT_MAX = 20.0     # Set same as CURRENT_LIMIT
CURRENT_HARD_MAX = 30.0     # Brief peaks only (1.5x continuous)
```

⚠️ **IMPORTANT**: Check your motor's rated current! Exceeding it will overheat the motor.

#### **Hall Sensor** (lines 56-57)
```python
HALL_POLARITY = 0           # Try 0 first, the script will detect correct value
HALL_POLARITY_CALIBRATED = False  # Set to True after first successful run
```

#### **Absolute Encoder** (lines 60-67)
```python
ENCODER_CPR = 16384         # From encoder datasheet
                            # 14-bit = 16384
                            # 13-bit = 8192
                            # 12-bit = 4096

ENCODER_MODE = ENCODER_MODE_SPI_ABS_CUI  # Options:
    # ENCODER_MODE_SPI_ABS_CUI (CUI AMT23x series)
    # ENCODER_MODE_SPI_ABS_AMS (AMS AS5047/AS5048)
    # ENCODER_MODE_INCREMENTAL (if using quadrature ABI)
```

**Common absolute encoders:**
- **CUI AMT232B**: 12-bit (4096 CPR) → use `ENCODER_MODE_SPI_ABS_CUI`
- **CUI AMT102**: Incremental (not absolute)
- **AMS AS5047**: 14-bit (16384 CPR) → use `ENCODER_MODE_SPI_ABS_AMS`
- **Broadcom AEAT-6012**: 12-bit SPI

#### **Gear Ratio** (lines 73-76)
```python
GEAR_RATIO = 10.0           # Motor shaft rotations per encoder rotation
```

**How to determine gear ratio:**
```
If gearbox is 10:1 and encoder is on OUTPUT shaft:
  → Motor spins 10 times, encoder spins 1 time
  → GEAR_RATIO = 10.0

If encoder is on MOTOR shaft (no gearbox):
  → GEAR_RATIO = 1.0
```

#### **Velocity Limits** (lines 79-81)
```python
VEL_LIMIT = 20.0            # Motor shaft speed in turns/s
                            # At 10:1 reduction, output = 2.0 turn/s
VEL_RAMP_RATE = 100.0       # Acceleration (turn/s²)
```

#### **CAN Bus** (lines 93-96)
```python
CAN_NODE_ID = 0             # Set unique ID for each ODrive (0-63)
CAN_BAUD_RATE = 250000      # Match your CAN bus bitrate
```

---

## 🚀 **Running the Calibration**

### **Step 1: Install ODrive Tools**
```bash
pip3 install odrive
```

### **Step 2: Connect ODrive**
- Connect ODrive S1 to computer via USB
- Power on ODrive with appropriate voltage (12V, 24V, or 48V)
- **⚠️ IMPORTANT**: Motor shaft must be **FREE to spin** during calibration!

### **Step 3: Run the Script**
```bash
cd /home/raj/BDR/pilot_ws/src/pilot_control/scripts
python3 odrive_s1_calib.py
```

Or specify ODrive by serial number:
```bash
python3 odrive_s1_calib.py 336A35623536
```

### **What Happens:**
1. ✅ Configures all parameters
2. 💾 Saves and reboots
3. 🔧 Runs motor calibration (measures R and L)
4. 🧭 Calibrates hall sensor polarity
5. 📐 Calibrates encoder offset
6. 💾 Saves calibration results
7. 🏃 Tests velocity control (3 second spin test)

---

## 📊 **Expected Output**

```
======================================================================
ODrive S1 Configuration & Calibration
======================================================================

🔍 Searching for ODrive...
✅ Connected to ODrive S1
   Firmware: v0.6.9
   Serial: 0x336A35623536
   Bus voltage: 24.12 V

--- Configuring Motor ---
✓ Motor type: MotorType.HIGH_CURRENT
✓ Pole pairs: 7
✓ Torque constant: 0.0488 N·m/A
✓ Current limit: 20.0A (calibration: 5.0A)

--- Configuring Hall Sensors (Commutation) ---
✓ Hall sensor enabled (polarity: 0)
✓ Set as commutation encoder

--- Configuring Absolute Encoder (Position Feedback) ---
✓ Found encoder: spi_encoder0
✓ Encoder mode: ENCODER_MODE_SPI_ABS_CUI
✓ Encoder CPR: 16384
✓ Gear ratio: 10.0:1
✓ Set as load encoder (position feedback)

... (calibration sequence) ...

======================================================================
CALIBRATION COMPLETE!
======================================================================

✅ ODrive is ready for operation!

💡 Next steps:
   1. Test manually: axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
   2. Set velocity: axis0.controller.input_vel = 2.0
   3. Set position: axis0.controller.input_pos = 10.0
```

---

## 🔍 **Troubleshooting**

### **Error: "Motor calibration failed"**
**Causes:**
- Motor shaft is blocked or loaded
- Calibration current too low
- Wrong motor parameters

**Solutions:**
1. Ensure motor can spin freely
2. Increase `CALIBRATION_CURRENT` to 10A
3. Check pole pairs and motor type

### **Error: "Hall calibration failed"**
**Causes:**
- Hall sensors not connected
- Wrong hall polarity
- Damaged hall sensors

**Solutions:**
1. Check hall sensor wiring (typically 5V, GND, HA, HB, HC)
2. Try both `HALL_POLARITY = 0` and `HALL_POLARITY = 6`
3. Check hall sensor output with oscilloscope

### **Error: "Encoder calibration failed"**
**Causes:**
- Encoder not connected
- Wrong encoder mode
- SPI communication issue

**Solutions:**
1. Check encoder wiring (MISO, MOSI, SCK, CS, VCC, GND)
2. Try different `ENCODER_MODE` settings
3. Verify encoder power supply (3.3V or 5V)

### **Motor spins opposite direction**
**Solution:**
```python
# Swap any two motor phases (A ↔ B or B ↔ C)
# OR change hall polarity:
HALL_POLARITY = 6  # if it was 0
```

### **Position drifts or jumps**
**Causes:**
- Incorrect gear ratio
- Encoder slipping on shaft

**Solutions:**
1. Double-check `GEAR_RATIO` calculation
2. Ensure encoder is mechanically secure

### **"Velocity limit violation" errors**
**Solution:**
```python
# Increase velocity limit in script:
VEL_LIMIT = 30.0  # or higher
```

---

## 🧪 **Manual Testing After Calibration**

### **Test in `odrivetool`:**
```bash
odrivetool
```

```python
# Arm the motor
odrv0.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL

# Test velocity control
odrv0.axis0.controller.input_vel = 1.0  # 1 turn/s
odrv0.axis0.controller.input_vel = 0.0  # stop

# Test position control
odrv0.axis0.controller.config.control_mode = ControlMode.POSITION_CONTROL
odrv0.axis0.controller.input_pos = 10.0  # move to 10 turns

# Check position
odrv0.axis0.encoder.pos_estimate  # current position (turns)

# Disarm
odrv0.axis0.requested_state = AxisState.IDLE
```

---

## 🔗 **Integration with ROS**

After calibration, your ODrive is ready for CAN bus control with your ROS nodes:

```bash
# In your launch file, ODrive node will connect via CAN
ros2 run odrive_can odrive_can_node --ros-args \
  -r __ns:=/motor \
  -p node_id:=0 \
  -p interface:=can0
```

Then use your position test script:
```bash
ros2 run pilot_control test_gpr_position.py
```

---

## 📝 **Notes**

1. **Calibration values are saved**: After running once, set:
   ```python
   STARTUP_MOTOR_CALIBRATION = False
   STARTUP_ENCODER_OFFSET_CAL = False
   ```

2. **Hall polarity**: After first calibration, the script will print detected polarity. Update script with that value.

3. **Gear ratio matters**: This affects position commands! If `GEAR_RATIO = 10.0`:
   - Commanding 1 turn moves the OUTPUT shaft 1 turn
   - Motor shaft actually spins 10 turns

4. **CAN vs USB**: After calibration via USB, you can control via CAN for production use.

---

## 🆘 **Support**

For ODrive-specific issues:
- ODrive Documentation: https://docs.odriverobotics.com/
- ODrive Discord: https://discord.com/invite/k3ZZ3mS

For this script:
- Check parameter values carefully
- Ensure motor can spin freely during calibration
- Verify encoder wiring and power

