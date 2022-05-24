#!/bin/bash

# Initialization.
USER_ID=${HOST_UID}
GROUP_ID=${HOST_GID}
NAME=${USER_NAME}

# Rename NAME.
if [[ -n "${NAME}" ]] && [[ "${NAME}" != "${DEFAULT_USER}" ]]; then
  usermod -l "${NAME}" "${DEFAULT_USER}"
else
  NAME=${DEFAULT_USER}
fi
echo "${NAME}   ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Set Host UID.
if [[ -n "${USER_ID}" ]]; then
  usermod -u "${USER_ID}" "${NAME}"
fi

# Set Host GID.
# Give Group Owner ID to Home directories.
if [[ -n "${GROUP_ID}" ]] && [[ "${GROUP_ID}" != "$(id -g ${NAME})" ]]; then
  groupadd -g "${GROUP_ID}" "${NAME}"
  usermod -g "${GROUP_ID}" "${NAME}"
fi
chown -R $(id -u ${NAME}):$(id -g ${NAME}) "/run/user/$(id -u ${NAME})"

# ==================================================================================================================
#
#   DEBUG
#
# ==================================================================================================================
DEBUG_NAME=${USER_NAME}
if [[ -z "${USER_NAME}" ]]; then
  DEBUG_NAME="unset (Default: ${DEFAULT_USER})"
fi

DEBUG_USER_ID=${USER_ID}
if [[ -z "${USER_ID}" ]]; then
  DEBUG_USER_ID="unset (Default: $(id "${NAME}" -u))"
fi

DEBUG_GROUP_ID=${GROUP_ID}
if [[ -z "${GROUP_ID}" ]]; then
  DEBUG_GROUP_ID="unset (Default: $(id "${NAME}" -g))"
fi

echo ""
echo "User Name: ${DEBUG_NAME}"
echo "User ID: ${DEBUG_USER_ID}"
echo "Group ID: ${DEBUG_GROUP_ID}"
echo "Login Command: $*"
echo
if [ "${DOCKER_RUNTIME}" = "nvidia" ]; then
  nvidia-smi
  echo
fi
echo ""
echo "Complete initialization of entrypoint"
exec "$@"
