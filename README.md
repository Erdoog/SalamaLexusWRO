# SalamaLexus Future Engineers WRO 2023
В этом репозитории наша команда собрала все необходимые файлы для документации. Чтобы просмотреть все файлы откройте папки с картинками роботов, 3d моделями, фотографиями команды, а также можете ознакомиться с файлом README, для уточняющей информации.

## Описание файлов и папок документации
``models`` - 3Д модели, разработанные в Fusion360, которые в последующем были распечатаны для использования в нашем роботе

``others`` - файлы необходимые для запуска программы на системной плате робота

``robot-photos`` - 6 фотографий робота

``team-photos`` - 2 фотографии команды (веселая и официальная)

``schemes`` - 2 схемы для сборки робота (электрическая и механическая)

``video`` - видео-обзор от нашей команды на платформе YouTube

``src`` - основная программа (код)

### Коротко о роботе (машинке):
Задний привод работает на моторе. Передняя рулевая ось подключена к серво-моторчику. Два ультразвуковых датчика и камера используются для ориентации на поле. Контрольная система состоит из Control Hub и Arduino nano. Весь корпус был сделан с нуля из фанеры и распечатанных на 3д принтере деталях. (файлы в папке models).

### Мобильность робота (Mobility Management)
>Для нашей команды важно добиться как высокого темпа при заездах, так и пройти все круги максимально аккуратно, чтобы не получить штрафных очков, поэтому робот получился относительно узким у задней оси, что позволит нам допускать меньше ошибок при поворотах и обходах препятствий. Привод робота - задний.

Помимо этого, мы решили убрать лишнее трение с задней оси, так как она неподвижна. Сделали мы это засчет удаления резины, сохраняя ее на передней оси для большего трениия и поворотности!

### Основная система/плата:
Благодаря предыдущему опыту использования, а также легкой интеграции в работу с другими платами (например Ардуино) наш выбор выпал на контрольную систему REV Robotics Control Hub. Данная плата содержит необходимые нам порты для подключения всех моторов и сенсоров, а также энкодеров (счетчики оборотов, проделанных моторами). Позже мы подключили ее к Ардуино Нано для использования дополнительных сенсоров из Ардуино наборов. Схема подключения находится в папке `schemes`

### Выбор моторов:
Наша задача не допустить слишком большой скорости, которую будет тяжело контролировать при объездах препятствий и поворотах, поэтому идеальный RPM (rotations per minute) между 100 и 150, и помимо этого масса не должна быть большой для возможности использовать более объемные аккумуляторы и другие компоненты. В нашей лаборатории были моторы **GoBilda и CoreHEX**. Сравнив характеристики, мы выбрали второй вариант, так как по всем главным сравнительным характеристикам он был намного лучше (125 RPM, Torque 3.2N-m): 

![alt text](https://cdn11.bigcommerce.com/s-t3eo8vwp22/images/stencil/1280x1280/products/195/2675/REV-41-1300_Core_Hex5_not_lm__05075.1661790332.png?c=2)

>Для управления передней осью (передними колесами) нет необходимости в отдельном полноценном моторе, поэтому серво-мотор экономит пространство и массу.

Вместо оригинального корпуса мотора мы решили использовать свой, распечатав его на 3д принтере (файл есть в папке  *models*)

### Питание и сенсоры:
Питание не требовало больших значений mAh, но определенно требовало от 11.1В до 12В напряжения, которое нужно для корректной работы контрольный системы (мозга робота), порт XT-30 и небольшой размер батареи был получен с помощью **Li-Po 11.1V 1500mAh**:

![alt text](https://ba3ar.kz/wp-content/uploads/2020/09/1_886.jpg)

*Единственные сенсоры, которые мы использовали это Ардуино ультра-звуковые сенсоры расстояния, а также камеру 720p Logitech для распознования зеленых и красных препятствий. Gyro sensor или же IMU уже встроен в Control Hub, что значительно упрощает процесс подключения дополнительных устройств и сенсоров.*

![alt text](https://hackster.imgix.net/uploads/attachments/1110572/_yN0cJOpsQ9.blob?auto=compress&w=900&h=675&fit=min&fm=jpg)

## Принцип кода
1. Камера, с помощью библиотеки OpenCV определяет кадр и регулярно на протяжении работы всей программы сохраняет данные о том, сколько процентов всех пикселей занимают пиксели красного или зеленого цвета, чтобы узнать какое препятствие из них ближе или дальше.

![alt text](https://github.com/Erdoog/SalamaLexusWRO/blob/master/readme/obstacle.png?raw=true)
   
3. Для построения конкретного движения используется алгоритм "RoadRunner" разработанный на платформе FIRST Tech Challenge и использовать который возможно только при помощи **Android Studio** (читайте ниже как установить Андроид Студио)
Основная идея "RoadRunner" заключается в том, чтобы создать комплексную программу, которая позволяет роботу автоматически и точно выполнять задачи на поле, используя данные из компьютерного зрения и закрытых петель управления. Этот подход может значительно повысить эффективность и надежность автономного периода в соревнованиях.

## Установка необходимого ПО для программирования робота
1. Android Studio — это продвинутая интегрированная среда разработки для создания приложений для Android. Этот инструмент является тем же инструментом, который используют профессиональные разработчики приложений для Android. Android Studio рекомендуется для продвинутых пользователей с большим опытом программирования на Java.

2.![alt text](https://github.com/Erdoog/SalamaLexusWRO/blob/master/readme/picture_1.png?raw=true)

Перейдите на сайт андроид-разработчика для загрузки и установки Android Studio:
https://developer.android.com/studio/index.html

3. Нажмите зеленую кнопку «СКАЧАТЬ ANDROID STUDIO», чтобы начать процесс загрузки.
После загрузки установочного пакета запустите приложение и следуйте инструкциям на экране для установки Android Studio.

![alt text](https://github.com/Erdoog/SalamaLexusWRO/blob/master/readme/picture_2.png?raw=true)

4. FTC SDK можно загрузить из репозитория GitHub. GitHub — веб-платформа по контролю версий.
которая позволяет отдельным лицам и организациям размещать контент в Интернете. Чтобы получить доступ к программному обеспечению FTC, вам потребуется публичный репозиторий текущего сезона можно найти по следующему адресу:
https://github.com/FIRST-Tech-Challenge/SkyStone

![alt text](https://github.com/Erdoog/SalamaLexusWRO/blob/master/readme/picture_3.png?raw=true)

5. Чтобы импортировать проект FTC, вам необходимо запустить программное обеспечение Android Studio на вашем компьютере. На главном экране Android Studio выберите параметр «Импортировать проект (Eclipse, ADT, Gradle и т. д.)», чтобы начать процесс импорта.

![alt text](https://github.com/Erdoog/SalamaLexusWRO/blob/master/readme/picture_4.png?raw=true)

6. Android Studio должна предложить вам выбрать папку проекта, которую вы хотите импортировать. Используйте файловый браузер во всплывающем диалоговом окне, чтобы найти и выбрать папку.

7. Убедитесь, что вы выбрали папку извлеченного проекта (а не файл .ZIP, который может иметь имя, похожее на
извлеченная папка). Нажмите кнопку «ОК», чтобы импортировать выбранный проект в Android Studio.

8. Для импорта в Android Studio выбрана папка проекта под названием «SkyStone-5.0». Это
Android Studio может потребоваться несколько минут для импорта проекта.
