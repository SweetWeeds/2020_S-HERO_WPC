"""sior URL Configuration

The `urlpatterns` list routes URLs to views. For more information please see:
    https://docs.djangoproject.com/en/2.2/topics/http/urls/
Examples:
Function views
    1. Add an import:  from my_app import views
    2. Add a URL to urlpatterns:  path('', views.home, name='home')
Class-based views
    1. Add an import:  from other_app.views import Home
    2. Add a URL to urlpatterns:  path('', Home.as_view(), name='home')
Including another URLconf
    1. Import the include() function: from django.urls import include, path
    2. Add a URL to urlpatterns:  path('blog/', include('blog.urls'))
"""
from django.contrib import admin
from django.urls import path
from . import views
urlpatterns = [
    path('', views.home),
    #path('location', views.location),
    #path('alert', views.alert),
    #path('statistics', views.statistics),
    #path('oauth/', views.oauth, name='oauth'), #added for kakao
    #path('alert', views.alert, name='alert'),
    #path('normal', views.normal, name='normal'),
    #path('device_list', views.device_list, name='device_list'),
    path('node_info/<slug:node_info>/', views.node_info, name='node_info'),
    path('node_info/<slug:node_info>/ps', views.parking_start, name='parking_start'),
    path('criteria', views.criteria, name='criteria')
]
