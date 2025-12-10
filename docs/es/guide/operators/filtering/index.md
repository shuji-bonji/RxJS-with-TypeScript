---
description: Los operadores de filtrado de RxJS se utilizan para extraer solo los datos necesarios del flujo basándose en condiciones y tiempo, contribuyendo a mejorar el rendimiento.
---

# Operadores de Filtrado

Los operadores de filtrado de RxJS son herramientas importantes para seleccionar solo los datos necesarios de un flujo y no dejar pasar datos innecesarios.
Esto mejora en gran medida la eficiencia y el rendimiento de la aplicación.

Los operadores de filtrado son un conjunto de operadores de RxJS que ordenan los valores en un flujo, permitiendo que solo pasen aquellos que cumplan ciertos criterios.
Al controlar el flujo de datos y procesar solo los valores que necesitas, puedes construir un canal de procesamiento de datos eficiente.


## Lista de Operadores
### ◾ Operadores de Filtrado Básicos

| Operador | Descripción |
|:---|:---|
| [filter](./filter) | Solo deja pasar valores que coincidan con la condición |
| [take](./take) | Obtiene solo el número especificado de primeros valores |
| [takeLast](./takeLast) | Obtiene el número especificado de últimos valores |
| [takeWhile](./takeWhile) | Obtiene valores mientras se cumpla la condición |
| [skip](./skip) | Omite el número especificado de primeros valores |
| [skipLast](./skipLast) | Omite el número especificado de últimos valores |
| [skipWhile](./skipWhile) | Omite valores mientras se cumpla la condición |
| [skipUntil](./skipUntil) | Omite valores hasta que otro Observable emita |
| [first](./first) | Obtiene el primer valor, o el primer valor que cumpla una condición |
| [last](./last) | Obtiene el último valor, o el último valor que cumpla una condición |
| [elementAt](./elementAt) | Obtiene el valor en un índice especificado |
| [find](./find) | Encuentra el primer valor que cumpla una condición |
| [findIndex](./findIndex) | Obtiene el índice del primer valor que cumpla una condición |
| [ignoreElements](./ignoreElements) | Ignora todos los valores y solo pasa completaciones/errores |


### ◾ Operadores de Filtrado Basados en Tiempo

| Operador | Descripción |
|:---|:---|
| [debounceTime](./debounceTime) | Emite el último valor si no se recibe entrada durante un tiempo especificado |
| [throttleTime](./throttleTime) | Pasa el primer valor e ignora nuevos valores durante el tiempo especificado |
| [auditTime](./auditTime) | Emite el último valor después de un tiempo especificado |
| [audit](./audit) | Controla el período con un Observable personalizado y emite el último valor |
| [sampleTime](./sampleTime) | Muestrea el último valor en intervalos de tiempo especificados |


### ◾ Operadores de Filtrado Basados en Condiciones

| Operador | Descripción |
|:---|:---|
| [distinct](./distinct) | Elimina todos los valores duplicados (emite solo valores únicos) |
| [distinctUntilChanged](./distinctUntilChanged) | Elimina valores duplicados consecutivos |
| [distinctUntilKeyChanged](./distinctUntilKeyChanged) | Detecta solo cambios en propiedades específicas |


## Casos de Uso Prácticos

- [Casos de Uso Prácticos](./practical-use-cases.md) presenta ejemplos prácticos de combinación de múltiples operadores de filtrado (búsqueda en tiempo real, desplazamiento infinito, etc.).
