---
description: Los operadores de utilidad son un grupo de operadores auxiliares en RxJS que se encargan de controlar efectos secundarios, procesamiento de retardo, gestión de suscripciones, etc.
---

# Operadores de Utilidad

Los operadores de utilidad en RxJS son un grupo de operadores que se encargan del **procesamiento auxiliar de streams (efectos secundarios, control de estado, soporte de UI, etc.)** en lugar del propósito principal de conversión o filtrado de datos.

En esta página, los operadores se clasifican por propósito como se muestra a continuación, y se proporciona una lista para confirmar su uso básico.
Para el uso detallado y ejemplos prácticos, consulte las páginas respectivas o [Casos de Uso Prácticos](./practical-use-cases.md).


## Lista de Operadores (por Propósito)

### ◾ Efectos Secundarios y Control de Estado

| Operador | Descripción | A menudo Combinado Con |
|--------------|------|------------------|
| [tap](./tap.md) | Ejecutar efectos secundarios sin cambiar valores (salida de log, actualizaciones de UI, etc.) | `map`, `switchMap` |
| [finalize](./finalize.md) | Ejecutar procesamiento de limpieza cuando termina el stream | `tap`, `catchError` |


### ◾ Control de Temporización y Retardo

| Operador | Descripción | A menudo Combinado Con |
|--------------|------|------------------|
| [delay](./delay.md) | Retrasar la emisión de cada valor por un tiempo especificado | `tap`, `concatMap` |
| [timeout](./timeout.md) | Generar un error si la emisión excede cierto tiempo | `catchError`, `retry` |
| [takeUntil](./takeUntil.md) | Finalizar suscripción cuando el Observable especificado notifica | `interval`, `fromEvent` |


### ◾ Valor Inicial, Repetición, Conversión a Array, etc.

| Operador | Descripción | A menudo Combinado Con |
|--------------|------|------------------|
| [startWith](./startWith.md) | Emitir un valor inicial al principio del stream | `scan`, `combineLatest` |
| [repeat](./repeat.md) | Resuscribirse al stream completo después de completarse | `tap`, `delay` |
| [retry](./retry.md) | Reintentar en caso de error | `catchError`, `switchMap` |
| [toArray](./toArray.md) | Emitir todos los valores en el stream como un solo array (al completarse) | `concatMap`, `take` |


## Observaciones

- Diferencia entre `retry` y `repeat`:
  - `retry`: **Reintentar en caso de error**
  - `repeat`: **Reintentar al completarse exitosamente**
- `toArray` no emite un valor a menos que se complete, por lo que comúnmente se usa con `take()` y similares.
