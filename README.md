# Digital Propeller 

## Descripción

Trabajo final para la materia **Digitales 2** de la **UNSAM** (Universidad Nacional de San Martín).  
**Año:** 2025 - Segundo Cuatrimestre  
**Autor:** Diego H. Mirarchi

Este proyecto implementa un digital propeller basado en el efecto de persistencia de visión (POV). Utiliza una barra vertical de 14 LEDs que, al girar, proyecta imágenes y texto en el aire mediante la sincronización precisa del encendido de los LEDs con la posición angular de rotación.

## Características Técnicas

- **Microcontrolador:** STM32F103 (Blue Pill)
- **LEDs:** 14 LEDs verticales controlados por GPIOA
- **Sincronización:** Detección por sensor hall SS49e con ADC de referencia en cada vuelta
- **Resolución angular:** 180 columnas por rotación completa (2° por columna)
- **Timer:** TIM2 configurado a 30 kHz para timestamping de rotación
