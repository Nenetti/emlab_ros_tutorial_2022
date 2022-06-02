#!/bin/bash

# ----------------------------------------------------------------------------------------------------------------------
#
#   Change user name
#
# ----------------------------------------------------------------------------------------------------------------------
function rename(){
  local _default_name="$1"
  local _new_name="$2"

  if [[ -z "${_new_name}" ]]; then
    echo "${_default_name}"
    return 0
  fi

  if [[ "${_new_name}" != "${_default_name}" ]]; then
    usermod -l "${_new_name}" "${_default_name}"
  fi
  echo "${_new_name}"
  return 0
}

# ----------------------------------------------------------------------------------------------------------------------
#
#   Set UID
#
# ----------------------------------------------------------------------------------------------------------------------
function set_user_id(){
  local _user_name="$1"
  local _user_id="$2"

  if [[ -n "${_user_id}" ]]; then
    usermod -u "${_user_id}" "${_user_name}"
  fi
  return 0
}

# ----------------------------------------------------------------------------------------------------------------------
#
#   Set GID
#
# ----------------------------------------------------------------------------------------------------------------------
function set_group_id(){
  local _user_name="$1"
  local _group_id="$2"

  if [[ -n "${_group_id}" ]] && [[ "${_group_id}" != "$(id -g ${_user_name})" ]]; then
    groupadd -g "${_group_id}" "${_user_name}"
    usermod -g "${_group_id}" "${_user_name}"
  fi
  chown -R $(id -u ${_user_name}):$(id -g ${_user_name}) "/run/user/$(id -u ${_user_name})"
}

# ----------------------------------------------------------------------------------------------------------------------
#
#   Assign sudo user
#
# ----------------------------------------------------------------------------------------------------------------------
function set_sudo_authority(){
  local _user_name="$1"

  echo "${_user_name}   ALL=(ALL) NOPASSWD:ALL" >>/etc/sudoers
}

# ----------------------------------------------------------------------------------------------------------------------
#
#   Change home directory
#
# ----------------------------------------------------------------------------------------------------------------------
function change_home_directory(){
  local _default_name="$1"
  local _new_name="$2"

  if [[ "${_new_name}" != "${_default_name}" ]]; then
    usermod -d "/home/${_new_name}" "${_new_name}"
    ln -s "/home/${_default_name}" "/home/${_new_name}"
  fi
}

# Initialization.
USER_ID=${HOST_UID}
GROUP_ID=${HOST_GID}
NEW_NAME=${USER_NAME}

NAME=$(rename ${DEFAULT_USER} ${NEW_NAME})
set_user_id "${NAME}" "${USER_ID}"
set_group_id "${NAME}" "${GROUP_ID}"
set_sudo_authority "${NAME}"
change_home_directory "${DEFAULT_USER}" "${NAME}"

# ----------------------------------------------------------------------------------------------------------------------
#
#   Print Debug
#
# ----------------------------------------------------------------------------------------------------------------------
if [[ -z "${USER_NAME}" ]]; then
  DEBUG_NAME="unset (Default: ${DEFAULT_USER})"
else
  DEBUG_NAME=${USER_NAME}
fi

if [[ -z "${USER_ID}" ]]; then
  DEBUG_USER_ID="unset (Default: $(id "${NAME}" -u))"
else
  DEBUG_USER_ID=${USER_ID}
fi

if [[ -z "${GROUP_ID}" ]]; then
  DEBUG_GROUP_ID="unset (Default: $(id "${NAME}" -g))"
else
  DEBUG_GROUP_ID=${GROUP_ID}
fi

echo ""
echo "User Name: ${DEBUG_NAME}"
echo "User ID: ${DEBUG_USER_ID}"
echo "Group ID: ${DEBUG_GROUP_ID}"
echo "Login Command: $*"
echo
echo "Complete initialization of entrypoint"
exec "$@"
