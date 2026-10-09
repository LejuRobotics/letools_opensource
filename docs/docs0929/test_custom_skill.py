import pytest

from core.domain.result import Result
from skills.templates.custom_skill.custom_skill import CustomSkill, CustomSkillParams


class FakeHardware:
    pass


@pytest.mark.unit
def test_custom_skill_finishes_without_hardware_side_effects():
    skill = CustomSkill(FakeHardware())
    assert skill.initialize(CustomSkillParams(value=1.0)).success
    result = skill.execute()
    assert isinstance(result, Result)
    assert result.success
    assert skill.is_finished()
