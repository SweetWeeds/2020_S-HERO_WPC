# Generated by Django 3.0.1 on 2020-01-04 21:21

from django.db import migrations, models


class Migration(migrations.Migration):

    dependencies = [
        ('dashboard', '0005_sensorvalue_co2'),
    ]

    operations = [
        migrations.CreateModel(
            name='isWarning',
            fields=[
                ('id', models.AutoField(auto_created=True, primary_key=True, serialize=False, verbose_name='ID')),
                ('isWarning', models.IntegerField(blank=True, default=0)),
            ],
        ),
    ]
