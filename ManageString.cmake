  macro (STRING_PAD _STR _LEN)
    set (_args ${ARGN})
    list (FIND _args "RIGHT" _RIGHT)
    if (_RIGHT EQUAL -1)
      set (_RIGHT)
    else (_RIGHT EQUAL -1)
      set (_RIGHT TRUE)
      list (REMOVE_ITEM _args "RIGHT")
    endif (_RIGHT EQUAL -1)
    # get maker string, split string at marker,
    # and make woring copy
    list (LENGTH _args _padding)
    if (_padding)
      list(GET _args 0 _padding)
      string_split (_split _padding ${_STR}
        NOESCAPE_SEMICOLON
        NOENCODE)
      if (_RIGHT)
        # need to right align 2nd part
        list (GET _split 1 _copy)
      else (_RIGHT)
        # left align 1st part
        list (GET _split 0 _copy)
      endif (_RIGHT)
    else (_padding)
      # no marker: copy complete string
      set (_copy ${${_STR}})
    endif (_padding)
    string (LENGTH ${_copy} _orig_len)
    if (_orig_len LESS ${_LEN})
      set (_big_space "                                                       ")
      if (_RIGHT)
        # right align: prepend space
        math (EXPR _extra_len "${_LEN} - ${_orig_len}")
        string (SUBSTRING ${_big_space} 0 ${_extra_len} _extra_space)
        set (_copy "${_extra_space}${_copy}")
      else (_RIGHT)
        set (_big_str "${_copy}${_big_space}")
        string (SUBSTRING ${_big_str} 0 ${_LEN} _copy)
      endif (_RIGHT)
    endif (_orig_len LESS ${_LEN})
    # join split string
    if (_padding)
      if (_RIGHT)
        # join copy w 1st part
        list (GET _split 0 _part)
        set (_copy "${_part}${_copy}")
      else (_RIGHT)
        # join copy w 2nd part
        list (GET _split 1 _part)
        set (_copy "${_copy}${_part}")
      endif (_RIGHT)
    endif (_padding)
    # replace string with working copy
    set (${_STR} ${_copy})
  endmacro (STRING_PAD _STR _LEN)
