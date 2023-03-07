from hassapi import Hass

hass = Hass(hassurl="http://192.168.1.64:8123/", token="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiI1OWI2YzM5MjE3NWY0YWMwOWM2NmY0MDcwZDdiY2FkYSIsImlhdCI6MTY3ODIyNTkxOSwiZXhwIjoxOTkzNTg1OTE5fQ.1eHXMscPPD1S8q9cCUnyN2BZ5wP5BMtSMqZ8s3zzgMA")

hass.turn_on("switch.aspirator")
#hass.run_script("hello")
