---
description: timeout es un operador de utilidad de RxJS que lanza un error si no se emite ningún valor desde Observable dentro de un tiempo especificado. Ideal para procesamiento reactivo con restricción de tiempo como control de tiempo de espera de solicitud de API, espera de respuestas de acción del usuario, o detección de retardo de stream. Se puede combinar con catchError para implementar comportamiento de respaldo, y la inferencia de tipos de TypeScript permite procesamiento de tiempo de espera con seguridad de tipos.
---

# timeout - Configuración de Tiempo de Espera

El operador `timeout` es un operador que **lanza un error si no se emite ningún valor por Observable dentro de un tiempo especificado**.
A menudo se utiliza para procesamiento reactivo, como esperar una respuesta a una solicitud de API u operación del usuario.


## 🔰 Sintaxis Básica y Operación

Si el tiempo de espera no se excede, la operación continúa normalmente; si excede cierto período, ocurre un error.

```ts
import { of } from 'rxjs';
import { delay, timeout, catchError } from 'rxjs';

of('respuesta')
  .pipe(
    delay(500), // 👈 Si se establece en 1500, muestra `Error de tiempo de espera: respaldo`
    timeout(1000),
    catchError((err) => of('Error de tiempo de espera: respaldo', err))
  )
  .subscribe(console.log);
// Salida:
// respuesta
```

En este ejemplo, `'respuesta'` se muestra normalmente ya que el valor se emite después de 500ms debido a `delay(500)` y se cumple la condición de `timeout(1000)`.

Si se especifica `delay(1200)`, se genera un `error de tiempo de espera` de la siguiente manera:
```sh
Error de tiempo de espera: respaldo
TimeoutErrorImpl {stack: 'Error\n    at _super (http://localhost:5174/node_mo…s/.vite/deps/chunk-RF6VPQMH.js?v=f6400bce:583:26)', message: 'Timeout has occurred', name: 'TimeoutError', info: {…}}
```

[🌐 Documentación Oficial de RxJS - timeout](https://rxjs.dev/api/index/function/timeout)

## 💡 Ejemplo de Uso Típico

El siguiente ejemplo muestra tanto un **patrón que causa tiempo de espera si el stream se retrasa y no emite un valor** como un **patrón que emite normalmente**.

```ts
import { interval, of } from 'rxjs';
import { timeout, catchError, take } from 'rxjs';

const slow$ = interval(1500).pipe(take(3));
const fast$ = interval(500).pipe(take(3));

fast$
  .pipe(
    timeout(1000),
    catchError((err) => of('respaldo: tiempo de espera ocurrido'))
  )
  .subscribe(console.log);

slow$
  .pipe(
    timeout(1000),
    catchError((err) => of('respaldo: tiempo de espera activado'))
  )
  .subscribe(console.log);
// Salida:
// 0
// 1
// respaldo: tiempo de espera activado
// 2
```


## 🧪 Ejemplo de Código Práctico (con UI)

```ts
import { interval, of } from 'rxjs';
import { timeout, catchError, take } from 'rxjs';

// Área de visualización de salida
const timeoutOutput = document.createElement('div');
timeoutOutput.innerHTML = '<h3>Ejemplo de timeout:</h3>';
document.body.appendChild(timeoutOutput);

// Caso de éxito de tiempo de espera
const normalStream$ = interval(500).pipe(take(5));

const timeoutSuccess = document.createElement('div');
timeoutSuccess.innerHTML = '<h4>Stream Normal (Sin Tiempo de Espera):</h4>';
timeoutOutput.appendChild(timeoutSuccess);

normalStream$
  .pipe(
    timeout(1000),
    catchError((err) => {
      const errorMsg = document.createElement('div');
      errorMsg.textContent = `Error: ${err.message}`;
      errorMsg.style.color = 'red';
      timeoutSuccess.appendChild(errorMsg);
      return of('Valor de respaldo después del error');
    })
  )
  .subscribe((val) => {
    const item = document.createElement('div');
    item.textContent = `Valor: ${val}`;
    timeoutSuccess.appendChild(item);
  });

// Caso de error de tiempo de espera
const slowStream$ = interval(1500).pipe(take(5));

const timeoutError = document.createElement('div');
timeoutError.innerHTML = '<h4>Stream Lento (Ocurre Tiempo de Espera):</h4>';
timeoutOutput.appendChild(timeoutError);

slowStream$
  .pipe(
    timeout(1000),
    catchError((err) => {
      const errorMsg = document.createElement('div');
      errorMsg.textContent = `Error: ${err.message}`;
      errorMsg.style.color = 'red';
      timeoutError.appendChild(errorMsg);
      return of('Valor de respaldo después del tiempo de espera');
    })
  )
  .subscribe((val) => {
    const item = document.createElement('div');
    item.textContent = `Valor: ${val}`;
    timeoutError.appendChild(item);
  });
```


## ✅ Resumen

- `timeout` es un operador de control que **lanza un error si no ocurre emisión dentro de cierto tiempo**
- Efectivo para procesamiento de tiempo de espera mientras se espera APIs de red u operaciones de UI
- Se puede combinar con `catchError` para especificar **comportamiento de respaldo**
