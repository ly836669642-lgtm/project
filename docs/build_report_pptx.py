"""Generates docs/Report.pptx: a 6-slide, ~7-minute talk summarizing
DOCUMENTATION.md (cover, system overview, planning, perception, control,
results), framed as three layers: Planning decides where to go, Perception
senses the world, Control drives and reacts in real time. Content and
figures are pulled directly from DOCUMENTATION.md / DOCUMENTATION.tex;
regenerate this script's output if the doc changes.

Run with: conda run -n claude python docs/build_report_pptx.py
"""

from pathlib import Path

from pptx import Presentation
from pptx.util import Inches, Pt, Emu
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from PIL import Image

DOCS = Path(__file__).parent
FIGS = DOCS / "figures"

SLIDE_W = Inches(13.333)
SLIDE_H = Inches(7.5)

NAVY = RGBColor(0x1B, 0x2A, 0x4A)
ACCENT = RGBColor(0x2E, 0x6F, 0x95)
DARK_TEXT = RGBColor(0x22, 0x22, 0x22)
GRAY_TEXT = RGBColor(0x55, 0x55, 0x55)
WHITE = RGBColor(0xFF, 0xFF, 0xFF)

MARGIN = Inches(0.5)
TITLE_TOP = Inches(0.3)
TITLE_H = Inches(0.85)
CONTENT_TOP = Inches(1.25)


def new_presentation():
    prs = Presentation()
    prs.slide_width = SLIDE_W
    prs.slide_height = SLIDE_H
    return prs


def blank_slide(prs):
    return prs.slides.add_slide(prs.slide_layouts[6])


def add_title(slide, text, size=32):
    box = slide.shapes.add_textbox(MARGIN, TITLE_TOP, SLIDE_W - 2 * MARGIN, TITLE_H)
    tf = box.text_frame
    tf.word_wrap = True
    p = tf.paragraphs[0]
    p.text = text
    p.font.size = Pt(size)
    p.font.bold = True
    p.font.color.rgb = NAVY
    # thin accent rule under the title
    rule = slide.shapes.add_shape(1, MARGIN, TITLE_TOP + TITLE_H - Inches(0.05), SLIDE_W - 2 * MARGIN, Pt(2.5))
    rule.fill.solid()
    rule.fill.fore_color.rgb = ACCENT
    rule.line.fill.background()
    rule.shadow.inherit = False
    return box


def add_bullets(slide, left, top, width, height, bullets, font_size=18, sub_size=15):
    box = slide.shapes.add_textbox(left, top, width, height)
    tf = box.text_frame
    tf.word_wrap = True
    first = True
    for item in bullets:
        if isinstance(item, tuple):
            text, level = item
        else:
            text, level = item, 0
        p = tf.paragraphs[0] if first else tf.add_paragraph()
        first = False
        p.text = ("•  " if level == 0 else "‒  ") + text
        p.level = 0
        p.font.size = Pt(font_size if level == 0 else sub_size)
        p.font.color.rgb = DARK_TEXT if level == 0 else GRAY_TEXT
        p.space_after = Pt(8 if level == 0 else 4)
    return box


def add_picture_fit(slide, path, left, top, max_w, max_h, caption=None):
    im = Image.open(path)
    iw, ih = im.size
    aspect = iw / ih
    box_aspect = max_w / max_h
    if aspect > box_aspect:
        w = max_w
        h = Emu(int(max_w / aspect))
    else:
        h = max_h
        w = Emu(int(max_h * aspect))
    x = Emu(int(left + (max_w - w) / 2))
    y = Emu(int(top + (max_h - h) / 2))
    slide.shapes.add_picture(str(path), x, y, width=w, height=h)
    if caption:
        cap_top = Emu(int(top + max_h + Inches(0.05)))
        box = slide.shapes.add_textbox(left, cap_top, max_w, Inches(0.3))
        tf = box.text_frame
        tf.word_wrap = True
        p = tf.paragraphs[0]
        p.text = caption
        p.font.size = Pt(12)
        p.font.italic = True
        p.font.color.rgb = GRAY_TEXT
        p.alignment = PP_ALIGN.CENTER
    return x, y, w, h


def set_notes(slide, text):
    slide.notes_slide.notes_text_frame.text = text


def build():
    prs = new_presentation()

    # ---------------- Slide 1: Cover ----------------
    s = blank_slide(prs)
    bg = s.shapes.add_shape(1, 0, 0, SLIDE_W, SLIDE_H)
    bg.fill.solid()
    bg.fill.fore_color.rgb = NAVY
    bg.line.fill.background()
    bg.shadow.inherit = False

    title_box = s.shapes.add_textbox(Inches(1.0), Inches(2.5), Inches(11.33), Inches(1.3))
    tf = title_box.text_frame
    tf.word_wrap = True
    p = tf.paragraphs[0]
    p.text = "Autonomous Driving Project"
    p.font.size = Pt(44)
    p.font.bold = True
    p.font.color.rgb = WHITE
    p.alignment = PP_ALIGN.CENTER

    sub_box = s.shapes.add_textbox(Inches(1.0), Inches(3.7), Inches(11.33), Inches(0.6))
    tf = sub_box.text_frame
    tf.word_wrap = True
    p = tf.paragraphs[0]
    p.text = 'TUM "Introduction to ROS" 2026 — Autonomous Driving Group Project'
    p.font.size = Pt(20)
    p.font.color.rgb = RGBColor(0xC9, 0xD6, 0xE8)
    p.alignment = PP_ALIGN.CENTER

    team_box = s.shapes.add_textbox(Inches(1.0), Inches(4.9), Inches(11.33), Inches(0.5))
    tf = team_box.text_frame
    tf.word_wrap = True
    p = tf.paragraphs[0]
    p.text = "Author 1  ·  Author 2  ·  Author 3"
    p.font.size = Pt(16)
    p.font.color.rgb = RGBColor(0x9B, 0xB0, 0xCC)
    p.alignment = PP_ALIGN.CENTER

    set_notes(s,
        "Good afternoon. This is our autonomous driving project for the Introduction "
        "to ROS course. I will walk through our system architecture, then planning, "
        "perception, and control, and finish with our test results."
    )

    # ---------------- Slide 2: System Overview ----------------
    s = blank_slide(prs)
    add_title(s, "System Overview")
    bullets = [
        "Fixed ~785 m urban loop; waypoints published once at startup",
        "5 custom + 2 course-provided ROS 2 packages",
        "Three layers, matching Figure 1:",
        ("Planning — decides destinations", 1),
        ("Perception — senses its surrounding environment", 1),
        ("Control — drives the car & reacts in real time", 1),
        "Benchmark run: 272 s, zero collisions, zero stalls, 1 detour, 2 correct light stops",
        ("Average speed 2.7 m/s, peak 7.5 m/s on the longest straights", 1),
    ]
    add_bullets(s, MARGIN, CONTENT_TOP, Inches(6.6), Inches(5.7), bullets, font_size=16, sub_size=14)

    img_left = Inches(7.35)
    img_col_w = Inches(5.5)
    add_picture_fit(s, FIGS / "architecture_illustrated.png", img_left, CONTENT_TOP,
                     img_col_w, Inches(3.35), "Fig. 1 — Node/data-flow architecture")

    set_notes(s,
        "Our car drives a fixed, roughly 785-meter urban loop, using waypoints "
        "published once at startup. We wrote five ROS 2 packages on top of two "
        "provided by the course, and we can think of the system in three layers, "
        "matching Figure 1. Planning decides where the car should go -- both the "
        "offline base route and live updates like rerouting around obstacles. "
        "Perception senses the world -- obstacles and traffic lights. And Control "
        "executes that path and reacts in real time -- steering, speed, and safety "
        "braking. Perception and planning both feed directly into control, and "
        "control is the only node that talks to the car itself. On the results "
        "side, a full benchmark run completed in 272 seconds with zero collisions, "
        "zero stalls, one obstacle detour, and two red-light stops handled "
        "correctly. Average speed was 2.7 meters per second, peaking at 7.5 on the "
        "two longest straights. Figure 5 shows the planned route on the course "
        "map, with the ten predefined goal poses our planner selects between."
    )

    # ---------------- Slide 3: Planning ----------------
    s = blank_slide(prs)
    add_title(s, "Planning — Deciding Destinations")
    bullets = [
        "Builds the path the car follows — offline, and online corrections while driving",
        "Offline at startup: smooth base path on the correct side of the road",
        ("sets a safe speed for every turn — tighter turns, lower speed", 1),
        "Online corrections: reactions to the world at the path level:",
        ("spots a parked obstacle → smoothly reroutes around it, then returns", 1),
        ("continuously picks the next checkpoint to aim for", 1),
        ("out of 10 fixed goal points placed around the loop", 1),
    ]
    add_bullets(s, MARGIN, CONTENT_TOP, SLIDE_W - 2 * MARGIN, Inches(5.9), bullets, font_size=21, sub_size=17)

    add_picture_fit(s, FIGS / "route_on_map.png", img_left, Inches(4.85),
                     img_col_w, Inches(2.1), "Fig. 5 — Route with 10 predefined goal poses")
    set_notes(s,
        "Planning is where the car decides where to go -- both offline and "
        "online. At startup, it builds a smooth base path that stays on the "
        "correct side of the road and sets a safe speed for every turn -- tighter "
        "turns get lower speeds. But it doesn't stop there: while driving, "
        "planning also reacts to the world at the path level. If control confirms "
        "a parked obstacle ahead, planning smoothly reroutes around it and "
        "returns to the normal path afterward. And it continuously picks the next "
        "checkpoint to aim for, out of ten fixed goal points placed around the "
        "loop."
    )

    # ---------------- Slide 4: Perception ----------------
    s = blank_slide(prs)
    add_title(s, "Perception — Sensing the World")
    bullets = [
        "Feeds real-time awareness to both planning and control",
        "Obstacle detection: depth camera builds a 3D view of the road ahead",
        ("tracks distance to the nearest obstacle, in the driving lane and to the sides", 1),
        ("ignores ground-level noise, only counts real obstacles", 1),
        ("backup 3D map check catches low, curb-height obstacles the camera misses", 1),
        "Traffic-light detection: RGB camera tells red from green",
        "Deliberately never uses the labeled (semantic) camera — full bonus earned",
        "Trade-off: detection updates only ~once a second, limiting safe driving speed",
    ]
    add_bullets(s, MARGIN, CONTENT_TOP, SLIDE_W - 2 * MARGIN, Inches(5.9), bullets, font_size=19, sub_size=16)

    set_notes(s,
        "Perception's job is simple: sense the world and hand that information to "
        "planning and control. An obstacle detector uses the depth camera to "
        "build a 3D view of the road ahead, and tracks how far away the nearest "
        "obstacle is, in the driving lane as well as to the sides. It ignores "
        "ground-level noise and only counts real obstacles, and a backup 3D map "
        "check catches low, curb-height obstacles the live camera might miss. A "
        "traffic-light detector uses color to tell red from green, and "
        "deliberately never uses the labeled semantic camera, which earns us the "
        "assignment's full bonus. One trade-off: detection only updates about "
        "once a second, which limits how fast the car can safely drive."
    )

    # ---------------- Slide 5: Control ----------------
    s = blank_slide(prs)
    add_title(s, "Control — Driving & Reacting in Real Time")
    bullets = [
        "Takes the path from planning and the readings from perception, drives the car",
        "Steering aims at a point ahead on the path (classic \"pursuit\" steering)",
        "Speed follows a safe distance from anything ahead — like adaptive cruise control",
        "Stops correctly at red lights, resumes on green",
        "When perception flags a parked obstacle → verifies it's clear, takes a smooth detour",
        "Independent emergency brake if a collision risk is detected",
        "Car is ready to drive automatically as soon as it starts up",
    ]
    add_bullets(s, MARGIN, CONTENT_TOP, SLIDE_W - 2 * MARGIN, Inches(5.9), bullets, font_size=19, sub_size=16)

    set_notes(s,
        "Control takes the path from planning and the live readings from "
        "perception, and drives the car -- steering and speed. Steering aims at a "
        "point on the path ahead, a classic and reliable method. Speed follows "
        "adaptive-cruise-control logic, keeping a safe distance from whatever "
        "perception detects ahead. On top of that, it stops correctly at red "
        "lights, and when perception flags a parked obstacle, control verifies "
        "the path is clear before taking a smooth detour around it. Independent "
        "of everything else, an emergency-brake check runs continuously and stops "
        "the car immediately if a collision risk is detected. And a simple switch "
        "means the car is ready to drive automatically as soon as it starts up."
    )

    # ---------------- Slide 6: Results ----------------
    s = blank_slide(prs)
    add_title(s, "Results")
    bullets = [
        "Fastest: two longest clear straights, peak 7.5 m/s",
        "Slowest (near-zero dips): detour spot, TL2/TL3 stops, 2 ACC slowdowns",
        ("ACC dips = Event I (merge) and Event II (crossing + hard brake, s≈531)", 1),
        "Design choice that mattered: wide-corridor cap 2.5 → 2.5–6.5 m/s ramp",
        ("roadside furniture occupies that corridor 60–90% of route time", 1),
        ("fixed route-wide starvation, not just one segment", 1),
        "Extra compute is localized to the detour segment (replan + re-profile)",
        "System-wide limiter: perception's ~0.7 Hz update rate bounds top speed",
    ]
    add_bullets(s, MARGIN, CONTENT_TOP, Inches(6.6), Inches(5.7), bullets, font_size=16, sub_size=14)

    img_left = Inches(7.35)
    img_col_w = Inches(5.5)
    add_picture_fit(s, FIGS / "route_speed_map.png", img_left, CONTENT_TOP,
                     img_col_w, Inches(3.1), "Fig. 3 — Driven path colored by speed")
    add_picture_fit(s, FIGS / "speed_profile.png", img_left, Inches(4.6),
                     img_col_w, Inches(2.3), "Fig. 4 — Speed vs. distance travelled")

    set_notes(s,
        "Figure 3 shows our driven path colored by speed, and Figure 4 shows that "
        "same speed against distance travelled. The fastest segments are the two "
        "longest clear straights, where we peak at 7.5 meters per second. The "
        "near-zero dips mark where we're slower: the detour location, the two "
        "traffic lights that were red on this run, and two ACC-managed slowdowns "
        "for the assignment's Event one and Event two encounters. One design "
        "choice mattered a lot here: we originally capped wide-corridor speed at a "
        "flat 2.5 meters per second, but roadside furniture sits in that corridor "
        "sixty to ninety percent of route time, so it was starving the whole run. "
        "Ramping that cap up to 6.5 meters per second fixed it route-wide, not "
        "just locally. The other limiter is perception's update rate -- capped "
        "around 0.7 hertz by the simulator's single-core loop -- which is what "
        "ultimately bounds how much faster we could safely go."
    )

    out_path = DOCS / "Report.pptx"
    prs.save(out_path)
    print(f"Saved {out_path}")


if __name__ == "__main__":
    build()
