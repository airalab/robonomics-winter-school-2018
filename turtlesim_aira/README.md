Пакет для AIRA для управления turtlesim через Робономику
========================================================

На период заключения договора кредитор может заказывать у turtilesim сервисы прохода по окружности, по часовой стрелке или против.

Установка и запуск сервиса
--------------------------

Система состоит из трёх частей: веб-интерфейс DApp, AIRA с пакетом `turtlesim_aira`, и робот, в нашем случае симуляция [turtlesim](http://wiki.ros.org/turtlesim).

1. Скачайте образ виртуальной машины с `turtlesim` [по ссылке](https://drive.google.com/file/d/1q-ANERcsjIOHZQSFJSOD6mbqSQmIMdT3), далее будем называть эту машину `hive`.

Пользователь | Пароль
-------------|-------
engi         | engi

> Пользователи GNU/Linux могут использовать `turtlesim` на локальной машине, установив `ROS` [по инструкции](http://wiki.ros.org/melodic/Installation) и пакет [turtlesim](http://wiki.ros.org/turtlesim). Для установки ROS в NixOS используйте канал [airapkgs](https://github.com/airalab/airapkgs).

2. Для подключения `hive` к AIRA создайте в VirtualBox виртуальную сеть между машинами.
Для этого в окне VirtualBox Manager: `Файл -> Менеджер сетей хоста`.
В появившемся окне нажмите `Создать`. В списке появится созданная сеть, в ней включить `DHCP сервер` установкой галочки в чекбокс в конце строки.

3. Импортируйте скачанный ранее образ `hive`.
4. Подключите AIRA к виртуальному адаптеру. Для этого в настройках машины перейдите в раздел `Сеть` и во вкладке `Адаптер 2` включите сетевой адаптер с типом подключения `Виртуальный адаптер хоста`.
5. Запустите обе виртуальные машины.

<details>
  <summary>Тормозит?</summary>
  
  Включите ускорение видео в настройках виртуальной машины.
  ![image](https://user-images.githubusercontent.com/833019/53819437-5e7c9780-3f83-11e9-8cd1-088dc8d93dea.png)
</details>

6. Теперь нужно сообщить машинам имена и IP-адреса друг друга. Для этого в терминалах каждой из машин наберите:
```console
ip a | awk '{print $2}' | egrep -o '192.168.56.[0-9]{,}'
```
И запишите полученные IP-адреса.
7. Теперь в конфигурации AIRA добавьте адрес `hive`. Откройте файл конфигурации:
```console
nano /etc/nixos/configuration.nix
```
Под строчкой `imports = [ ... ];` с новой строки добавьте:
```console
networking.extraHosts = "<IP Адрес hive> hive";
networking.hostName = "aira";
networking.firewall.enable = false;
```
Закройте текстовый редактор с сохранением изменений `Ctrl-x, y` и запустите новую конфигурацию:
```console
nixos-rebuild switch --fast
```
8. Теперь добавьте AIRA в `hive`. Для этого в терминале `hive`:
```console
sudo nano /etc/hosts
```
В файле найдите строчку вида `<IP адрес> aira` и замените IP-адрес на адрес Вашего `hive`.

9. Проверьте наличие связи между машинами. Для этого в `hive` запустите:
```console
ping aira
```
Если всё сделано правильно, каждую секунду будет появляться сообщение с временем прохода пакета от `hive` к AIRA.
То же самое из AIRA в `hive`:
```console
ping hive
```
В случае проблем обратитесь к инженерам в чате школы.

10. Устанвите и запустите `turtlesim_aira` в AIRA. Для этого в терминале AIRA:
```console
cd robonomics-winter-school-2018
git pull
nix build -f ./release.nix turtlesim_aira
source ./result/setup.zsh
roslaunch turtlesim_aira trader.launch
```
И в другом окне:
```console
source ./result/setup.zsh
roslaunch turtlesim_aira worker.launch
```

11. Подключите `hive` к `rosmaster` в AIRA и запустите `turtlesim`. Для этого в терминале `hive`:
```console
export ROS_MASTER_URI=http://aira:11311
rosrun turtlesim turtlesim_node
```

Готово! Теперь AIRA подключена к симуляции робота в `turtlesim`.

Параметры
---------

IPFS хэш                                       | Имя файла
-----------------------------------------------|-------------------------------------------------------
QmPVr7k4N2jNiCYjbvQWPcmxzm5jwY3ZHEuJMgbQLmPKvY | turtlesim_aira_order_allow.model
QmPtwRTjPmvBweSmG4zVGtUc9KWxLsPp76xERvjUXFJWEz | turtlesim_aira_order_duration_1h.objective
QmbYXWWhNtnjhhBTvs2UfHiFLTUsXSZsoLoKSufiZHPxvR | turtlesim_aira_order_duration_24h.objective
QmYijVc27M27WyS1UiAB72GmeDBKVo2Nyvh1EYXUBZUNJb | turtlesim_aira_order_duration_60s.objective
QmNeMoBUiYjk4VzLtsBe9XAXfpyFawsUd9wEYTQy4tZpEj | turtlesim_aira_order_move.model
QmRmj9VnRBbgmQwZMVU3oCinaYG8oh1UAvQJbtPUmEWSq1 | turtlesim_aira_order_circle_clockwise.objective
Qmd1YREP5MMLzoxT2kmvEocPxFMGFiCrLK6zQRmp5ebBqU | turtlesim_aira_order_circle_counterclockwise.objective

Параметр      | Значение
--------------|--------------------------------------------------------------------------------
lighthouse    | airalab.lighthouse.4.robonomics.sid, 0xE85764E29583224C1D063639d2AeeeD7c389DF4d
token         | xrt.4.robonomics.sid, 0x093ac06910f23570292fd5027a4fA558ed4Cd010

Содержание отчетов
------------------

### Сервисный контракт

Поле / топик | Тип           | Значение
-------------|---------------|-------------------------------------
success	     | std_msgs/Bool | False - не исполнен, True - исполнен

### Заказ

Поле / топик | Тип             | Значение
-------------|-----------------|-------------------------------------
success	     | std_msgs/Bool   | False - не исполнен, True - исполнен
/id          | std_msgs/String | IPFS ID исполнителя
/objective   | std_msgs/String | IPFS хеш objective заказа
