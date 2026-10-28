"""Factory for the S4b skills (search, come_to, return); `skills.make_skill` delegates here."""

from __future__ import annotations

from .come_to import ComeToSkill, ReturnSkill
from .search import SearchSkill
from .skills import SkillConfig, _Base


def make_extra_skill(req: dict, cfg: SkillConfig, now: float) -> _Base:
    kind = req.get("skill")
    if kind == "search":
        obj = str(req.get("object", "")).strip()
        if not obj:
            raise ValueError("search needs object")
        region = req.get("region")
        budget = req.get("budget_s")
        return SearchSkill(
            cfg,
            now,
            obj,
            str(region).strip().lower() if region else None,
            float(budget) if budget is not None else 120.0,
        )
    if kind == "come_to":
        who = str(req.get("person", "")).strip()
        if not who:
            raise ValueError("come_to needs person")
        return ComeToSkill(cfg, now, who, float(req.get("standoff_m", 1.3)))
    if kind == "return":
        home = req.get("home_xy")
        if not home:
            raise ValueError("return needs home_xy (the node adds it)")
        return ReturnSkill(cfg, now, (home[0], home[1]), bool(req.get("land", False)))
    raise ValueError(f"unknown skill {kind!r}")
