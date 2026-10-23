from Construct_Resource_Grid import SignalType

_NAME_TO_SIGTYPE = {
    "Ref_P0": SignalType.REF_P0,
    "Ref_P1": SignalType.REF_P1,
    "Data":   SignalType.DATA,
    "DC":     SignalType.DC,
}


def RG_Get_Number_Of_SigTypes(sig_type_name):
    """
    Return the number assigned to a signal type.

    sig_type_name : 'Ref_P0', 'Ref_P1', 'Data' or 'DC'
    returns       : Ref_P0 -> 10, Ref_P1 -> 20, DC -> 30, Data -> 40
    """
    try:
        return _NAME_TO_SIGTYPE[sig_type_name].value
    except KeyError:
        raise ValueError(
            f"sig_type_name must be one of {list(_NAME_TO_SIGTYPE)}, got {sig_type_name!r}"
        ) from None


if __name__ == "__main__":
    for name in ("Ref_P0", "Ref_P1", "Data", "DC"):
        print(name, RG_Get_Number_Of_SigTypes(name))
    try:
        RG_Get_Number_Of_SigTypes("Bogus")
    except ValueError as e:
        print("OK:", e)
