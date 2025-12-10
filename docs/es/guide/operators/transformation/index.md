---
description: Explica cómo procesar y transformar datos en un flujo utilizando operadores de transformación de RxJS, desde transformaciones simples como map, scan, mergeMap, switchMap y concatMap hasta transformaciones asíncronas, almacenamiento en búfer y ventanas. Se introducirán patrones prácticos que aprovechan la seguridad de tipos de TypeScript con abundantes ejemplos de código.
---

# Operadores de Transformación

Los operadores de transformación se utilizan para transformar y procesar datos dentro del pipeline de RxJS.
Al transformar valores en nuevas formas, permiten un control más flexible y potente sobre el flujo de datos reactivos.


## Lista de Operadores
### ◾ Transformaciones de Valores Simples

|Operador|Descripción|
|---|---|
|[map](./map)|Aplicar una función de transformación a cada valor|

### ◾ Acumulación

|Operador|Descripción|
|---|---|
|[scan](./scan)|Generar valores acumulativamente|
|[reduce](./reduce)|Emitir solo el resultado acumulado final|

### ◾ Pares y Agrupación

|Operador|Descripción|
|---|---|
|[pairwise](./pairwise)|Procesar dos valores consecutivos en pares|
|[groupBy](./groupBy)|Agrupar valores según una clave|

### ◾ Transformación Asíncrona

|Operador|Descripción|
|---|---|
|[mergeMap](./mergeMap) |Transformar cada valor en un Observable y fusionar en paralelo|
|[switchMap](./switchMap) |Cambiar al Observable más reciente|
|[concatMap](./concatMap) |Ejecutar cada Observable secuencialmente|
|[exhaustMap](./exhaustMap) |Ignorar nuevas entradas mientras se ejecuta|
|[expand](./expand) |Expandir recursivamente los resultados|

### ◾ Procesamiento por Lotes

|Operador|Descripción|
|---|---|
|[buffer](./buffer) |Agrupar valores en el momento de otro Observable|
|[bufferTime](./bufferTime) |Agrupar valores a intervalos regulares|
|[bufferCount](./bufferCount) |Agrupar valores por cantidad especificada|
|[bufferWhen](./bufferWhen) |Almacenamiento en búfer con condiciones de finalización controladas dinámicamente|
|[bufferToggle](./bufferToggle) |Almacenamiento en búfer con control independiente de inicio y fin|
|[windowTime](./windowTime) |Dividir en sub-Observables a intervalos regulares|


## Patrones de Transformación Prácticos

En aplicaciones del mundo real, el siguiente procesamiento es posible combinando operadores de transformación:

- Validación de entrada y retroalimentación
- Control óptimo de solicitudes API asíncronas
- Conformación, agregación y normalización de datos
- Procesamiento por lotes y agrupación de flujos de eventos

👉 Para más información: [Patrones de Transformación Prácticos](./practical-use-cases)

## 🚨 Notas

Para evitar errores comunes al usar operadores de transformación, consulte también:

- **[Efectos secundarios en map](/es/guide/anti-patterns/common-mistakes#5-efectos-secundarios-en-map)** - Usar `map` como una función pura
- **[Selección inadecuada de operadores](/es/guide/anti-patterns/common-mistakes#12-selección-inadecuada-de-operadores)** - Uso adecuado de operadores de orden superior
