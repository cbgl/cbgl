#!/bin/sh

# https://github.com/sudo-bmitch/docker-base/blob/main/bin/entrypointd.sh
# Copyright: Brandon Mitchell
# License: MIT

set -e
trap "{ exit 0; }" TERM INT

if [ $# -gt 1 ] && [ "$1" = "/bin/sh" ] && [ "$2" = "-c" ]; then
  shift 2
  eval "set -- $1"
fi

if [ -f /.volume-cache/volume-list.already-run ]; then
  rm /.volume-cache/volume-list.already-run
fi

for ep in /etc/entrypoint.d/*; do
  ext="${ep##*.}"
  if [ "${ext}" = "env" ] && [ -f "${ep}" ]; then
    echo "Sourcing: ${ep} $@"
    set -a && . "${ep}" "$@" && set +a
  elif [ "${ext}" = "sh" ] && [ -x "${ep}" ]; then
    echo "Running: ${ep} $@"
    "${ep}" "$@"
  fi
done

if [ -d /etc/certs.d ]; then
  add-certs
fi

if [ -f /.volume-cache/volume-list -a ! -f /.volume-cache/volume-list.already-run ]; then
  load-volume -a
fi

if [ -n "$ORIG_ENTRYPOINT" ]; then
  set -- "$ORIG_ENTRYPOINT" "$@"
fi

if [ $# = 0 ]; then
  if [ -x /bin/bash ]; then
    set -- /bin/bash
  else
    set -- /bin/sh
  fi
fi

if [ -n "${USE_INIT}" ]; then
  set -- tini -- "$@"
fi

USER=cbgl

chown --dereference $USER "/proc/$$/fd/1" "/proc/$$/fd/2" || :

chmod -R o+rwX /home/$USER/ros2_ws/src

chmod -R o+rwX /home/$USER/shared-input
chmod -R o+rwX /home/$USER/shared-output

set -- gosu $USER "$@"

exec "$@"
