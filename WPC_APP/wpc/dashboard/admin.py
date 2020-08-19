from django.contrib import admin
from dashboard.models import Charger, Node
#from dashboard.models import SensorValue, isWarning
from accounts.models import Token
# Register your models here.

admin.site.register(Charger)
admin.site.register(Node)
#admin.site.register(Token)
#admin.site.register(isWarning)