import attr


@attr.s(auto_attribs=True)
class RefereeCommand:
    state: int
    commandForTeam: int
    isPartOfFieldLeft: bool
