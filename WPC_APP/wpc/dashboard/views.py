from django.shortcuts import render
from django.shortcuts import redirect
from .models import Charger, Node
from django.db.models import Max
import requests
import json
from accounts.models import Token
import roslibpy
#from mbed_cloud import ConnectAPI
client = roslibpy.Ros(host='172.31.51.111', port=9090)
goal = roslibpy.Topic(client, 'goal', 'std_msgs/String')

# Create your views here.
def isauth(request):
    if request.user.is_authenticated:
        #print('auth')
        return redirect('../home')
    else:
        #print('no')
        return redirect('accounts/login')

def home(request):
    #kakao 때문에 추가함.
    #Hats = Hat.objects
    #여기부터 날씨 때문에 추가함.
    url = 'http://api.openweathermap.org/data/2.5/weather?q={}&units=imperial&appid=08a543cc6623032ae7fe6365a5c9b994'
    city = 'Republic of Korea'
    city_weather = requests.get(url.format(city)).json() #request the API data and convert the JSON to Python data types

    weather = {
        'city' : city,
        'temperature' : round((city_weather['main']['temp']-32)*(5/9),2), #섭씨 -> 화씨 (0°F − 32) × 5/9 
        'description' : city_weather['weather'][0]['description'],
        'icon' : city_weather['weather'][0]['icon']
    }
    #여기까지 내가 추가함.

    #SensorValues = SensorValue.objects
    #max_temperature = SensorValue.objects.order_by('-recordtime')[:10].aggregate(Max('temperature'))
    #max_voc = SensorValue.objects.order_by('-recordtime')[:10].aggregate(Max('voc'))
    #max_humid = SensorValue.objects.order_by('-recordtime')[:10].aggregate(Max('humid'))
    #max_co2 = SensorValue.objects.order_by('-recordtime')[:10].aggregate(Max('co2'))
    #max_air_quality = SensorValue.objects.order_by('-recordtime')[:10].aggregate(Max('air_quality'))
    if not request.user.is_authenticated:
        return redirect('../accounts/login')
    else:
        return render(request, 'home.html', {})

def node_info(request, node_info):
    node = Node.objects.all().get(node_name = node_info)
    #SensorValues = SensorValue.objects.all().filter(owner=hat)
    return render(request, 'node_info.html', {'node':node})

def parking_start(request, node_info):
    node = Node.objects.all().get(node_name = node_info)
    if True:
        if not client.is_connected:
            client.run()
        goal.publish(roslibpy.Message({'data':node_info}))
        #time.sleep(1)
        #goal.unadvertise()
        #client.close()
    return render(request, 'parking_start.html', {'node':node})

def criteria(request):
    return render(request, 'criteria.html', {'isWarning':isWarning.objects.last().isWarning})