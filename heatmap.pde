/* ==========================================================
   Gait Heatmap + IMU (MPU6050) — FINAL
   Your features enabled: (1) Step count  (3) Roll bar  (4) Pitch hint
   ----------------------------------------------------------
   ESP32 CSV (10 Hz):
   ts_ms,heel,arch,fore,toe,pitch_deg,roll_deg,yaw_deg
   ----------------------------------------------------------
   KEYS:  R = toggle reference image
          [ / ] = ref opacity
          G = toggle handles
          1/2/3/4 = select Toe/Fore/Arch/Heel (drag with mouse)
          S = save shape to data/foot_shape.txt
          L = load shape
   Notes:
   - Step detection uses total FSR force with adaptive high/low thresholds,
     so it works immediately without accel magnitude.
   - Roll bar: + = pronation (inward), − = supination (outward)
   - Pitch hint: heel-strike likely (pitch < -5°), toe-down likely (pitch > +5°)
   ========================================================== */
   // ---- Right panel layout (neater spacing) ----
int PANEL_W = 360;        // fixed width for the metrics panel
int PANEL_X;              // computed each frame
int PANEL_Y = 90;
int PANEL_H;              // computed each frame
int UI_PAD  = 12;         // inner padding
int UI_GAP  = 12;         // gap between cards
int TITLE_BAR_H = 26;     // title strip height


import processing.serial.*;
Serial port;
String PORT_NAME = "COM3";   // <- change this if needed
int    BAUD      = 115200;
boolean DEBUG_LINES = false;

/* -------- Layout (left = heatmap, right = IMU+metrics) -------- */
float cx, cy, fw, fh; // center, width, height (px) for foot box

/* -------- Reference image overlay (optional) -------- */
PImage refImg = null;   // put data/foot_ref.png if you have one
boolean showRef = true;
float refAlpha = 140;   // 0..255

/* -------- Foot outline (normalized 0..1) -------- */
ArrayList<PVector> footPts = new ArrayList<PVector>();

/* -------- Sensor anchors (normalized; x=0 medial-left, y=0 toe .. 1 heel) -------- */
PVector toePt  = new PVector(0.62, 0.10);  // 1: Toe
PVector forePt = new PVector(0.58, 0.30);  // 2: Forefoot
PVector archPt = new PVector(0.40, 0.58);  // 3: Arch
PVector heelPt = new PVector(0.50, 0.88);  // 4: Heel
int selectedSensor = 0; // 1..4 via keys

/* -------- Heatmap buffers -------- */
float sigmaRel = 0.16;
PGraphics heatPG, maskPG;

/* -------- Incoming data -------- */
float heel=0, arch=0, fore=0, toe=0;
float pitchDeg = 0, rollDeg = 0, yawDeg = 0;
long  lastRxMs = 0;

/* -------- Autoscale for heatmap -------- */
float dynMax = 150, decay = 0.99;

/* -------- Step detection (from FSR total force) -------- */
float totalForce = 0;
float dynMaxTotal = 600;       // adaptive “max” for total force
float dynDecayTotal = 0.995;
float highThresh = 180;        // adaptive; updated each frame
float lowThresh  = 90;         // adaptive; updated each frame
boolean inSwing = true;        // true when foot is “light”
int stepCount = 0;
final int REFRACTORY_MS = 250; // ignore re-triggers within this window
long lastStepMs = -999999;

/* -------- Cadence (SPM) from recent step intervals -------- */
final int STEP_HISTORY = 10;
float[] stepIntervals = new float[STEP_HISTORY]; // seconds between steps
int     stepIdx = 0;
boolean stepBufFilled = false;
float   cadenceSPM = 0;

/* -------- Roll bar config -------- */
float rollBarRangeDeg = 15; // ±15°
/* Pitch hint thresholds */
float PITCH_TOE_DOWN = 5;
float PITCH_HEEL_DOWN = -5;

/* -------- UI toggles -------- */
boolean showHandles = true;
int    dragIndex = -1;
boolean draggingSensor = false;

void setup() {
  pixelDensity(1);
  size(1200, 820, P2D);
  textFont(createFont("SansSerif", 14, true));
  smooth(4);

  // Foot box placement (left side)
  cx = width  * 0.40;
  cy = height * 0.55;
  fw = 560;
  fh = 900;

  // Heat buffers
  heatPG = createGraphics(int(fw), int(fh), P2D);
  maskPG = createGraphics(int(fw), int(fh), P2D);

  // Outline and saved shape
  initDefaultFoot();
  loadShapeFromFile();

  // Reference overlay (optional)
  try {
    refImg = loadImage("foot_ref.png");
    if (refImg != null) refImg.resize(int(fw), int(fh));
  } catch(Exception e) { refImg = null; }

  // Serial
  try {
    port = new Serial(this, PORT_NAME, BAUD);
    port.clear();
  } catch(Exception e) {
    println("⚠ Could not open " + PORT_NAME + ": " + e.getMessage());
  }
}

void draw() {
  background(246);

  /* -------- Read serial lines -------- */
  while (port != null && port.available() > 0) {
    String line = port.readStringUntil('\n');
    if (line == null) break;
    line = trim(line.replace("\r",""));
    if (line.length()==0) continue;
    if (line.startsWith("#")) continue; // ignore comments
    if (DEBUG_LINES) println("RX: "+line);
    if (parseCSVorFlexible(line)) lastRxMs = millis();
  }

  /* -------- Update autoscale & thresholds -------- */
  totalForce = heel + arch + fore + toe;

  // Autoscale per-sensor (for heat colors)
  float localMax = max(max(heel, arch), max(fore, toe));
  if (localMax > dynMax) dynMax = localMax; else dynMax *= decay;

  // Autoscale for step thresholds (total force)
  if (totalForce > dynMaxTotal) dynMaxTotal = totalForce;
  else dynMaxTotal *= dynDecayTotal;

  // Adaptive dual thresholds (works across shoes/weights)
  highThresh = max(120, 0.35 * dynMaxTotal);
  lowThresh  = 0.5 * highThresh;

  /* -------- Step detection (debounced) -------- */
  long now = millis();
  if (inSwing) {
    if (totalForce > highThresh && (now - lastStepMs) > REFRACTORY_MS) {
      // Heel-down / contact detected → step
      stepCount++;
      if (lastStepMs > 0) {
        float dt_s = (now - lastStepMs) / 1000.0;
        stepIntervals[stepIdx] = dt_s;
        stepIdx = (stepIdx + 1) % STEP_HISTORY;
        if (stepIdx == 0) stepBufFilled = true;
        // cadence using mean of recent intervals
        int n = stepBufFilled ? STEP_HISTORY : stepIdx;
        float sum = 0;
        for (int i=0;i<n;i++) sum += stepIntervals[i];
        float mean = max(1e-3, sum / n);
        cadenceSPM = 60.0 / mean;
      }
      lastStepMs = now;
      inSwing = false;
    }
  } else {
    if (totalForce < lowThresh) {
      inSwing = true;
    }
  }

  /* -------- DRAW: Left panel = foot heatmap -------- */
  if (showRef && refImg!=null) {
    tint(255, refAlpha);
    imageMode(CENTER);
    image(refImg, cx, cy);
    noTint();
  }

  // Foot outline
  stroke(60); strokeWeight(2); noFill();
  beginShape();
  for (PVector np : footPts) curveVertex(mapX(np.x), mapY(np.y));
  for (int i=0;i<3;i++) curveVertex(mapX(footPts.get(i).x), mapY(footPts.get(i).y));
  endShape();

  // Heatmap clipped to outline
  float nToe  = toe  / max(1, dynMax);
  float nFore = fore / max(1, dynMax);
  float nArch = arch / max(1, dynMax);
  float nHeel = heel / max(1, dynMax);
  drawHeatInto(heatPG, new float[]{nToe, nFore, nArch, nHeel});
  drawFootMaskInto(maskPG);
  PImage heatImg = heatPG.get();
  PImage maskImg = maskPG.get();
  heatImg.mask(maskImg);
  imageMode(CENTER);
  image(heatImg, cx, cy);

  // COP dot (AP axis)
  float sumF = totalForce;
  float copY = 0.0;
  if (sumF > 1e-4) {
    copY = (toe*toePt.y + fore*forePt.y + arch*archPt.y + heel*heelPt.y)/sumF;
  }
  drawCOPDot(0.5, copY);

  // sensor pins
  drawSensorDot(toePt,   color(220, 40, 40), selectedSensor==1);
  drawSensorDot(forePt,  color(220, 40, 40), selectedSensor==2);
  drawSensorDot(archPt,  color(220, 40, 40), selectedSensor==3);
  drawSensorDot(heelPt,  color(220, 40, 40), selectedSensor==4);

  if (showHandles) drawHandles();

  /* -------- DRAW: Right panel = IMU & metrics -------- */
  drawRightPanel();

  /* -------- Footer tips -------- */
  fill(0);
  text(String.format("Heel: %.0f   Arch: %.0f   Fore: %.0f   Toe: %.0f   (dynMax: %.0f)",
        heel, arch, fore, toe, dynMax), 20, height-50);

  if (millis() - lastRxMs > 2000) {
    text("No serial yet? Close Arduino Serial Monitor, confirm port, 115200 baud.", 20, 24);
  }
}

/* ===================== Right Panel (IMU + metrics) ===================== */
void drawRightPanel() {
  // compute right panel rect
  PANEL_X = width - PANEL_W - 60;
  PANEL_H = height - PANEL_Y*2;

  // outer panel
  stroke(90); fill(252);
  rect(PANEL_X, PANEL_Y, PANEL_W, PANEL_H);

  int x = PANEL_X + UI_PAD;
  int y = PANEL_Y + UI_PAD;
  int w = PANEL_W  - UI_PAD*2;

  // Title bar
  drawTitleBar(x, y, w, "IMU & Gait Metrics");
  y += TITLE_BAR_H + UI_GAP;

  // Cards (neatly stacked)
  y += cardRollBar(x, y, w, 88);          // roll bar
  y += UI_GAP;
  y += cardPitch(x, y, w, 70);            // pitch hint
  y += UI_GAP;
  y += cardSteps(x, y, w, 110);           // steps & cadence
  y += UI_GAP;
  y += cardAngles(x, y, w, 84);           // raw angles
}

/* ---------- small UI helpers ---------- */
void drawTitleBar(int x, int y, int w, String title){
  noStroke(); fill(240);
  rect(x, y, w, TITLE_BAR_H);
  fill(30); textAlign(LEFT, CENTER);
  text(title, x + 8, y + TITLE_BAR_H/2.0);
}

void drawCardFrame(int x, int y, int w, int h, String title){
  stroke(150); fill(255);
  rect(x, y, w, h);
  // section title
  fill(40); noStroke();
  textAlign(LEFT, TOP);
  text(title, x + 8, y + 8);
}

/* ---------- cards ---------- */
int cardRollBar(int x, int y, int w, int h){
  drawCardFrame(x, y, w, h, "Pronation / Supination (Roll)");

  int bx = x + 10;
  int by = y + 34;   // room under title
  int bw = w - 20;

  // baseline + zero tick
  stroke(170);
  line(bx, by, bx + bw, by);
  stroke(140);
  line(bx + bw/2, by - 10, bx + bw/2, by + 10);

  // knob
  float norm = constrain(rollDeg / rollBarRangeDeg, -1, 1); // -1..+1
  int curX = int(lerp(bx, bx + bw, (norm + 1) * 0.5));
  stroke(0); fill(235);
  ellipse(curX, by, 14, 14);

  // labels
  fill(60);
  textAlign(LEFT, BOTTOM);  text("Supination", bx, by - 6);
  textAlign(RIGHT, BOTTOM); text("Pronation",  bx + bw, by - 6);
  textAlign(CENTER, TOP);
  text(String.format("roll = %.1f°   (range ±%.0f°)", rollDeg, rollBarRangeDeg),
       bx + bw/2, by + 10);

  return h;
}

int cardPitch(int x, int y, int w, int h){
  drawCardFrame(x, y, w, h, "Pitch (Heel strike vs Toe down)");

  String hint = "neutral";
  if (pitchDeg <= PITCH_HEEL_DOWN) hint = "heel strike likely";
  else if (pitchDeg >= PITCH_TOE_DOWN) hint = "toe-down likely";

  fill(30); textAlign(LEFT, TOP);
  text(String.format("pitch = %.1f°   →   %s", pitchDeg, hint), x + 10, y + 34);
  return h;
}

int cardSteps(int x, int y, int w, int h){
  drawCardFrame(x, y, w, h, "Steps & Cadence (FSR)");

  // stats
  fill(30); textAlign(LEFT, TOP);
  text("steps: " + stepCount, x + 10, y + 34);
  text(String.format("cadence: %.1f SPM", cadenceSPM), x + 10, y + 56);

  // force bar
  float frac = constrain(totalForce / max(1, highThresh * 2), 0, 1);
  int barW = int((w - 20) * frac);
  int bx = x + 10;
  int by = y + h - 22;
  noStroke(); fill(70, 185, 90);
  rect(bx, by, barW, 10);
  stroke(160); noFill();
  rect(bx, by, w - 20, 10);
  fill(70); textAlign(RIGHT, BOTTOM);
  text("force", x + w - 10, by - 2);

  return h;
}

int cardAngles(int x, int y, int w, int h){
  drawCardFrame(x, y, w, h, "Angles");
  fill(30); textAlign(LEFT, TOP);
  text(String.format("roll  :  %.1f°", rollDeg),  x + 10, y + 34);
  text(String.format("pitch :  %.1f°", pitchDeg), x + 10, y + 56);
  text(String.format("yaw* :  %.1f°  (not fused)", yawDeg), x + 10, y + 78);
  return h;
}


int drawRollBar(int x, int y, int w, int h) {
  // Frame
  stroke(90); noFill(); rect(x, y, w, h);
  fill(20); text("Pronation / Supination (Roll)", x+8, y+8);

  // Bar baseline
  int barY = y + h/2;
  int barL = x + 10;
  int barR = x + w - 10;
  stroke(150); line(barL, barY, barR, barY);

  // Zero mark
  stroke(120); line((barL+barR)/2, barY-10, (barL+barR)/2, barY+10);

  // Map roll to displacement
  float r = rollDeg;
  float norm = constrain(r / rollBarRangeDeg, -1, 1); // -1..+1
  int curX = int(lerp(barL, barR, (norm + 1) * 0.5));
  stroke(0); fill(230);
  ellipse(curX, barY, 14, 14);

  // Labels
  fill(0);
  textAlign(LEFT, CENTER);  text("Supination", barL, barY - 18);
  textAlign(RIGHT, CENTER); text("Pronation",  barR, barY - 18);
  textAlign(CENTER, TOP);   text(String.format("roll = %.1f°  (range ±%.0f°)", r, rollBarRangeDeg), (barL+barR)/2, barY+8);

  return h;
}

int drawPitchBox(int x, int y, int w, int h) {
  stroke(90); noFill(); rect(x, y, w, h);
  fill(20); text("Pitch (Heel strike vs Toe down)", x+8, y+8);

  String hint = "neutral";
  if (pitchDeg <= PITCH_HEEL_DOWN) hint = "heel strike likely";
  else if (pitchDeg >= PITCH_TOE_DOWN) hint = "toe-down likely";

  fill(0);
  textAlign(LEFT, TOP);
  text(String.format("pitch = %.1f°  →  %s", pitchDeg, hint), x+10, y+32);

  return h;
}

int drawStepsBox(int x, int y, int w, int h) {
  stroke(90); noFill(); rect(x, y, w, h);
  fill(20); text("Steps & Cadence (FSR-based)", x+8, y+8);

  fill(0);
  textAlign(LEFT, TOP);
  text("steps: " + stepCount, x+10, y+32);
  text(String.format("cadence: %.1f SPM", cadenceSPM), x+10, y+54);

  // tiny sparkline of total force (not stored; simple instant bar)
  float frac = 0;
  float denom = max(1, highThresh * 2);
  frac = constrain(totalForce / denom, 0, 1);
  int barW = int((w-20) * frac);
  noStroke(); fill(60,180,80);
  rect(x+10, y+h-18, barW, 10);

  stroke(140); noFill();
  rect(x+10, y+h-18, w-20, 10);
  fill(0); textAlign(RIGHT, BOTTOM);
  text("force", x+w-10, y+h-20);

  return h;
}

int drawAnglesBox(int x, int y, int w, int h) {
  stroke(90); noFill(); rect(x, y, w, h);
  fill(20); text("Angles", x+8, y+8);
  fill(0); textAlign(LEFT, TOP);
  text(String.format("roll  = %.1f°",  rollDeg),  x+10, y+30);
  text(String.format("pitch = %.1f°",  pitchDeg), x+10, y+52);
  text(String.format("yaw*  = %.1f° (not fused)", yawDeg), x+10, y+74);
  return h;
}

int drawThresholdBox(int x, int y, int w, int h) {
  stroke(90); noFill(); rect(x, y, w, h);
  fill(20); text("Thresholds (debug)", x+8, y+8);
  fill(0); textAlign(LEFT, TOP);
  text(String.format("total=%.0f  dynMaxTotal=%.0f", totalForce, dynMaxTotal), x+10, y+30);
  text(String.format("high=%.0f  low=%.0f", highThresh, lowThresh), x+10, y+52);
  return h;
}

/* ===================== Serial parsing ===================== */
boolean parseCSVorFlexible(String line) {
  try {
    if (line.indexOf(',') >= 0) {
      String[] t = split(line, ',');
      // Expect at least 8 columns per your Arduino code
      if (t.length >= 8 && isNumeric(t[0])) {
        // t[0] = ts_ms (ignored here)
        heel     = float(t[1]);
        arch     = float(t[2]);
        fore     = float(t[3]);
        toe      = float(t[4]);
        pitchDeg = float(t[5]);
        rollDeg  = float(t[6]);
        yawDeg   = float(t[7]); // not used for metrics
        return true;
      }
      // Fallback: if someone prints only fsr CSV
      if (t.length >= 5 && isNumeric(t[0])) {
        heel=float(t[1]); arch=float(t[2]); fore=float(t[3]); toe=float(t[4]);
        return true;
      }
    }

    // Back-compat with label lines (not used in your ESP32 code)
    if (line.startsWith("Heel"))      { heel = valAfterColon(line); return true; }
    if (line.startsWith("Arch"))      { arch = valAfterColon(line); return true; }
    if (line.startsWith("Forefoot"))  { fore = valAfterColon(line); return true; }
    if (line.startsWith("Toe"))       { toe  = valAfterColon(line); return true; }
    if (line.startsWith("FSR1"))      { heel = valAfterColon(line); return true; }
    if (line.startsWith("FSR2"))      { arch = valAfterColon(line); return true; }
    if (line.startsWith("FSR3"))      { fore = valAfterColon(line); return true; }
    if (line.startsWith("FSR4"))      { toe  = valAfterColon(line); return true; }
  } catch(Exception e) { /* ignore malformed */ }
  return false;
}

float valAfterColon(String s){
  int i=s.indexOf(':'); if (i>=0 && i+1<s.length()){
    String v=trim(s.substring(i+1)); if (isNumeric(v)) return float(v);
  } return 0;
}
boolean isNumeric(String s){
  if (s==null||s.length()==0) return false;
  for (int i=0;i<s.length();i++){
    char c=s.charAt(i);
    if (!((c>='0'&&c<='9')||c=='-'||c=='+'||c=='.')) return false;
  } return true;
}

/* ===================== Heat & Mask ===================== */
void drawHeatInto(PGraphics pg, float[] nv) {
  pg.beginDraw();
  pg.noStroke();
  pg.background(0,0);
  float sigma = sigmaRel * min(fw, fh);
  float twoSigma2 = 2*sigma*sigma;

  PVector[] centers = new PVector[]{
    footToPg(toePt), footToPg(forePt), footToPg(archPt), footToPg(heelPt)
  };

  pg.loadPixels();
  for (int y=0; y<pg.height; y++){
    for (int x=0; x<pg.width; x++){
      float accum=0;
      for (int i=0;i<centers.length;i++){
        float dx=x-centers[i].x, dy=y-centers[i].y;
        float w=exp(-(dx*dx+dy*dy)/twoSigma2);
        accum += nv[i]*w;
      }
      float v = constrain(accum,0,1);
      int c1=pg.color(0,255,0), c2=pg.color(255,255,0), c3=pg.color(255,0,0);
      int col=(v<0.5)? lerpColor(c1,c2,v*2.0) : lerpColor(c2,c3,(v-0.5)*2.0);
      pg.pixels[y*pg.width + x] = col;
    }
  }
  pg.updatePixels();
  pg.endDraw();
}

void drawFootMaskInto(PGraphics pg){
  pg.beginDraw();
  pg.background(0); pg.noStroke(); pg.fill(255);
  pg.beginShape();
  PVector p0 = footPts.get(0);
  pg.curveVertex(footToPgX(p0.x), footToPgY(p0.y));
  for (PVector np : footPts) pg.curveVertex(footToPgX(np.x), footToPgY(np.y));
  for (int i=0;i<3;i++) {
    PVector np = footPts.get(i);
    pg.curveVertex(footToPgX(np.x), footToPgY(np.y));
  }
  pg.endShape(CLOSE);
  pg.endDraw();
}

/* ===================== Interaction ===================== */
void mousePressed(){
  if (!showHandles) return;

  // if sensor selected via 1/2/3/4 and cursor near it → drag sensor
  PVector sel = getSelectedSensor();
  if (selectedSensor>0){
    PVector sp = toScreen(sel);
    if (dist(mouseX,mouseY, sp.x,sp.y) < 18){
      draggingSensor = true;
      return;
    }
  }
  // else pick nearest outline point
  float best=18; dragIndex=-1;
  for (int i=0;i<footPts.size();i++){
    PVector sp = toScreen(footPts.get(i));
    float d = dist(mouseX,mouseY, sp.x,sp.y);
    if (d<best){ best=d; dragIndex=i; }
  }
}
void mouseDragged(){
  if (!showHandles) return;

  if (draggingSensor && selectedSensor>0){
    PVector np = fromScreen(mouseX, mouseY);
    np.x = constrain(np.x, 0, 1); np.y = constrain(np.y, 0, 1);
    setSelectedSensor(np);
    return;
  }
  if (dragIndex>=0){
    PVector np = fromScreen(mouseX, mouseY);
    np.x = constrain(np.x, 0, 1); np.y = constrain(np.y, 0, 1);
    footPts.set(dragIndex, np);
  }
}
void mouseReleased(){ dragIndex=-1; draggingSensor=false; }

void keyPressed(){
  if (key=='r'||key=='R') showRef=!showRef;
  if (key=='[') refAlpha=max(0, refAlpha-15);
  if (key==']') refAlpha=min(255, refAlpha+15);
  if (key=='g'||key=='G') showHandles=!showHandles;

  if (key=='1') selectedSensor=1;
  if (key=='2') selectedSensor=2;
  if (key=='3') selectedSensor=3;
  if (key=='4') selectedSensor=4;

  if (key=='s'||key=='S') saveShapeToFile();
  if (key=='l'||key=='L') loadShapeFromFile();

  // Optional: change roll bar range
  if (key=='+') rollBarRangeDeg = min(30, rollBarRangeDeg+1);
  if (key=='-') rollBarRangeDeg = max(5,  rollBarRangeDeg-1);
}

/* ===================== Drawing helpers ===================== */
void drawHandles(){
  stroke(30,120); fill(255);
  for (int i=0;i<footPts.size();i++){
    PVector sp = toScreen(footPts.get(i));
    ellipse(sp.x, sp.y, 10, 10);
  }
  // label selected sensor
  fill(0);
  if (selectedSensor>0) text("Dragging sensor: "+selectedSensor+"  (1=Toe, 2=Fore, 3=Arch, 4=Heel)", 20, 40);
}
void drawSensorDot(PVector np, int col, boolean sel){
  PVector sp = toScreen(np);
  stroke(0); fill(col);
  ellipse(sp.x, sp.y, sel?16:12, sel?16:12);
}
void drawCOPDot(float nx, float ny){
  PVector sp = toScreen(new PVector(nx, ny));
  stroke(0); fill(220,40,40);
  ellipse(sp.x, sp.y, 12, 12);
}

/* ===================== Mapping ===================== */
float mapX(float nx){ return cx + (nx-0.5)*fw; }
float mapY(float ny){ return cy + (ny-0.5)*fh; }
PVector toScreen(PVector np){ return new PVector(mapX(np.x), mapY(np.y)); }
PVector fromScreen(float x, float y){ return new PVector((x-cx)/fw+0.5, (y-cy)/fh+0.5); }
PVector footToPg(PVector np){ return new PVector((np.x-0.5f)*fw+fw/2f, (np.y-0.5f)*fh+fh/2f); }
float footToPgX(float nx){ return (nx-0.5f)*fw+fw/2f; }
float footToPgY(float ny){ return (ny-0.5f)*fh+fh/2f; }

/* ===================== Shape I/O ===================== */
void initDefaultFoot(){
  footPts.clear();
  // outer heel → toe → inner heel (right foot), 24 pts
  float[][] d = {
    {0.30,0.92},{0.28,0.85},{0.28,0.78},{0.30,0.70},{0.33,0.62},{0.36,0.55},
    {0.38,0.50},{0.39,0.45},{0.39,0.40},{0.41,0.34},{0.45,0.27},{0.51,0.22},
    {0.60,0.16},{0.70,0.14},{0.78,0.17},{0.82,0.22},{0.83,0.29},{0.82,0.38},
    {0.80,0.50},{0.78,0.62},{0.74,0.75},{0.68,0.86},{0.60,0.94},{0.50,0.98}
  };
  for (float[] p : d) footPts.add(new PVector(p[0], p[1]));
}

void saveShapeToFile(){
  String[] lines = new String[footPts.size()+5];
  lines[0] = "# foot_shape.txt (normalized points + sensors)";
  int idx=1;
  for (PVector p : footPts){
    lines[idx++] = String.format("P %.6f %.6f", p.x, p.y);
  }
  lines[idx++] = String.format("S toe %.6f %.6f",  toePt.x,  toePt.y);
  lines[idx++] = String.format("S fore %.6f %.6f", forePt.x, forePt.y);
  lines[idx++] = String.format("S arch %.6f %.6f", archPt.x, archPt.y);
  lines[idx++] = String.format("S heel %.6f %.6f", heelPt.x, heelPt.y);
  saveStrings(dataPath("foot_shape.txt"), lines);
  println("✔ Saved to data/foot_shape.txt");
}
void loadShapeFromFile(){
  String fn = dataPath("foot_shape.txt");
  java.io.File f = new java.io.File(fn);
  if (!f.exists()) return;
  String[] ls = loadStrings(fn);
  if (ls==null) return;
  ArrayList<PVector> newPts = new ArrayList<PVector>();
  for (String s : ls){
    if (s==null || s.startsWith("#")) continue;
    String[] t = splitTokens(s, " \t");
    if (t.length>=3 && t[0].equals("P")){
      newPts.add(new PVector(constrain(float(t[1]),0,1), constrain(float(t[2]),0,1)));
    } else if (t.length>=4 && t[0].equals("S")){
      PVector v = new PVector(constrain(float(t[2]),0,1), constrain(float(t[3]),0,1));
      if (t[1].equals("toe"))  toePt=v;
      if (t[1].equals("fore")) forePt=v;
      if (t[1].equals("arch")) archPt=v;
      if (t[1].equals("heel")) heelPt=v;
    }
  }
  if (newPts.size()>=8){ footPts=newPts; println("✔ Loaded foot_shape.txt"); }
}

/* ===================== Sensor select helpers ===================== */
PVector getSelectedSensor(){
  if (selectedSensor==1) return toePt;
  if (selectedSensor==2) return forePt;
  if (selectedSensor==3) return archPt;
  if (selectedSensor==4) return heelPt;
  return new PVector(-1,-1);
}
void setSelectedSensor(PVector np){
  if (selectedSensor==1) toePt=np;
  if (selectedSensor==2) forePt=np;
  if (selectedSensor==3) archPt=np;
  if (selectedSensor==4) heelPt=np;
}