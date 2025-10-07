# Velocity Estimation in Dual-Encoder Setup

## 🎯 **Your Hardware Configuration**

```
Motor Shaft (High Speed)
    ↓
Hall Sensors (6-state, coarse)
    ↓
[GEARBOX - 10:1 reduction with backlash]
    ↓
Output Shaft (Low Speed, High Torque)
    ↓
AMT212B-OD (12-bit absolute, 4096 CPR, precise)
```

## ⚙️ **Two Encoder Roles**

| Role | Encoder | Location | Purpose |
|------|---------|----------|---------|
| **Commutation** | Hall sensors | Motor shaft | Tell motor controller which coils to energize |
| **Position (Load)** | AMT212B-OD | Output shaft | Precise position feedback for control |

## 🤔 **The Problem: Which Encoder for Velocity?**

ODrive needs a velocity estimate (`vel_estimate`) for:
- Velocity control mode
- Velocity feed-forward in position control
- All velocity-based control algorithms

### **Option 1: Load Encoder (Default)**
```python
use_commutation_vel = False
```

**Pros:**
- ✅ More accurate (12-bit vs 6-state)
- ✅ Velocity matches position encoder
- ✅ Better for very smooth, precise velocity control

**Cons:**
- ❌ **Gearbox backlash** affects velocity measurement
- ❌ Delayed response (signal goes through gearbox)
- ❌ Can cause oscillations or instability
- ❌ Problems during direction changes

### **Option 2: Commutation Encoder (Recommended for Gearboxes)**
```python
use_commutation_vel = True
commutation_vel_scale = 1.0 / GEAR_RATIO  # e.g., 1.0 / 10.0 = 0.1
```

**Pros:**
- ✅ **No backlash** (direct motor shaft measurement)
- ✅ Instant response (no mechanical delay)
- ✅ More stable velocity control
- ✅ Better for dynamic applications
- ✅ **Recommended by ODrive docs for gearbox setups**

**Cons:**
- ❌ Less precise (6-state hall vs 4096 CPR)
- ❌ Noisier signal at low speeds

---

## 📊 **How It Works**

### **Why Pole Pairs Matter**

Hall sensors measure **electrical rotations**, not mechanical rotations!

For a motor with 20 pole pairs:
- 1 mechanical rotation = 20 electrical rotations (hall sensor cycles)
- Hall sensors "see" 20× more rotations than the motor shaft actually makes

### **The Correct Formula (from ODrive docs)**

```python
commutation_vel_scale = 1 / (pole_pairs × N)
```

Where:
- `pole_pairs` = number of motor pole pairs (e.g., 20)
- `N` = gear ratio (e.g., 10:1 → N = 10)

### **Example Calculation**

Your setup:
- Pole pairs: 20
- Gear ratio: 10:1
- Formula: `1 / (20 × 10) = 1 / 200 = 0.005`

```
Hall sensors: 200 electrical rotations/s
    ↓
    ÷ 20 pole pairs = 10 mechanical rotations/s (motor shaft)
    ↓
    ÷ 10 gear ratio = 1 rotation/s (output shaft)
    ↓
    Combined: × 0.005 scale factor
    ↓
vel_estimate: 1 turn/s (output shaft speed)
```

---

## 🎯 **Best Practice for Your Setup**

### **Recommended Configuration**
```python
USE_COMMUTATION_VEL = True      # Use hall sensors
COMMUTATION_VEL_SCALE = 0.1     # For 10:1 gearbox
```

**Why?**
1. ✅ Your gearbox likely has backlash (all do)
2. ✅ Hall sensors give instant, backlash-free velocity
3. ✅ Position control still uses precise SPI encoder
4. ✅ **Best of both worlds!**

### **Result**
- **Position**: Precise AMT212B-OD (4096 CPR)
- **Velocity**: Responsive hall sensors (no backlash)
- **Commutation**: Hall sensors (their primary job)

---

## 🧪 **Testing Both Options**

### **Test 1: Load Encoder Velocity (Default)**
```python
USE_COMMUTATION_VEL = False
```

**What to watch for:**
- Oscillations during velocity control
- Instability when changing direction
- "Hunting" behavior (constant small corrections)

### **Test 2: Commutation Encoder Velocity (Recommended)**
```python
USE_COMMUTATION_VEL = True
COMMUTATION_VEL_SCALE = 1.0 / GEAR_RATIO
```

**What to watch for:**
- Smoother velocity response
- Better stability
- Less hunting behavior
- Slightly noisier at very low speeds

---

## 🔧 **Configuration Examples**

### **Example 1: Your Setup (20 pole pairs, 10:1 Gearbox)**
```python
POLE_PAIRS = 20
GEAR_RATIO = 10.0
USE_COMMUTATION_VEL = True
COMMUTATION_VEL_SCALE = 1.0 / (20 * 10.0)  # = 0.005
```

### **Example 2: 7 pole pairs, 5:1 Gearbox**
```python
POLE_PAIRS = 7
GEAR_RATIO = 5.0
USE_COMMUTATION_VEL = True
COMMUTATION_VEL_SCALE = 1.0 / (7 * 5.0)  # = 0.0286
```

### **Example 3: Direct Drive (No Gearbox)**
```python
POLE_PAIRS = 20
GEAR_RATIO = 1.0
USE_COMMUTATION_VEL = True  # Can still use, or use load encoder
COMMUTATION_VEL_SCALE = 1.0 / (20 * 1.0)  # = 0.05
# OR
USE_COMMUTATION_VEL = False  # No backlash, use precise encoder
```

---

## 📝 **Verification**

### **Check Current Settings**
```bash
odrivetool
```

```python
# Check if using commutation velocity
odrv0.axis0.controller.config.use_commutation_vel
# Should be: True

# Check scale factor
odrv0.axis0.controller.config.commutation_vel_scale
# Should be: 0.005 (for 20 pole pairs, 10:1 gearbox)
# Formula: 1 / (20 × 10) = 0.005

# Check velocity estimate
odrv0.axis0.encoder.vel_estimate  # This is what controller uses
```

### **Test Velocity Accuracy**
```python
# Spin motor at known speed
odrv0.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
odrv0.axis0.controller.input_vel = 1.0  # 1 turn/s OUTPUT speed

# Check estimates
hall_vel = odrv0.hall_encoder0.vel_estimate  # Should be ~10 turn/s (motor)
spi_vel = odrv0.spi_encoder0.vel_estimate    # Should be ~1 turn/s (output)
controller_vel = odrv0.axis0.encoder.vel_estimate  # What controller uses

# With USE_COMMUTATION_VEL = True:
# controller_vel should equal spi_vel (both ~1 turn/s)
```

---

## 🎯 **Summary**

| Aspect | Load Encoder | Commutation Encoder |
|--------|--------------|---------------------|
| **Accuracy** | High (4096 CPR) | Medium (6-state) |
| **Backlash** | ❌ Affected | ✅ None |
| **Response** | Slower | Faster |
| **Stability** | Can oscillate | More stable |
| **Use Case** | Direct drive, no backlash | **Gearbox with backlash** |
| **ODrive Recommendation** | For direct drive | **For gearboxes** ✅ |

**For your setup**: ✅ **Use `USE_COMMUTATION_VEL = True`**

---

## 📚 **References**

From ODrive Documentation:
> "In some cases (for example in a dual-encoder setup where the load encoder 
> sits behind a gearbox with backlash) it is more suitable to use the 
> commutation encoder for velocity estimation. To do this, set 
> `use_commutation_vel` to `True` and configure `commutation_vel_scale`."

**Key takeaway**: Your setup is EXACTLY the use case ODrive describes!

---

## 💡 **Practical Impact**

### **Before (Load Encoder)**
```
You command: 1.0 turn/s
Motor responds: Oscillates around 1.0 turn/s
Reason: Backlash causes velocity estimate to lag
Result: Controller overcorrects → instability
```

### **After (Commutation Encoder)**
```
You command: 1.0 turn/s
Motor responds: Stable at 1.0 turn/s
Reason: Hall sensors give instant feedback
Result: Smooth, stable velocity control ✅
```

---

## ⚙️ **Your Configuration**

The script `odrive_s1_configured.py` is set to:

```python
USE_COMMUTATION_VEL = True
COMMUTATION_VEL_SCALE = 1.0 / (POLE_PAIRS * GEAR_RATIO)  # Correct formula!

# For your setup (20 pole pairs, 10:1 gearbox):
COMMUTATION_VEL_SCALE = 1.0 / (20 * 10) = 0.005

# This means:
# - Position: From SPI encoder (precise)
# - Velocity: From hall sensors (responsive, no backlash)
# - Commutation: From hall sensors (their job)
# - Scale factor accounts for both pole pairs AND gear ratio
```

**This is the optimal configuration for your hardware!** 🎉

### **Why Pole Pairs Matter**

The key insight: **Hall sensors measure electrical cycles, not mechanical rotations!**

- 1 mechanical motor rotation = `pole_pairs` electrical cycles
- Your motor: 20 pole pairs means 20 electrical cycles per rotation
- Combined with 10:1 gearbox: 200 electrical cycles per output shaft rotation
- Scale factor: `1/200 = 0.005` converts electrical→output velocity

