import pygame as pg


def make_mono_font(size: int = 14) -> pg.font.Font:
    font_name = 'mono'
    fonts = [x for x in pg.font.get_fonts() if font_name in x]
    default_font = 'ubuntumono'
    mono = default_font if default_font in fonts else fonts[0]
    mono = pg.font.match_font(mono)
    return pg.font.Font(mono, size)
