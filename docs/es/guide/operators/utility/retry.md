---
description: El operador retry resuscribe y reintenta la fuente un número especificado de veces cuando ocurre un error en Observable. Esto es útil para recuperarse de fallos de comunicación temporales, como fallos de red, o para procesos que pueden tener éxito si se reintentan después de un fallo.
---

# retry - Reintentar en Caso de Error

El operador `retry` es un operador que **resuscribe el Observable fuente un número especificado de veces** cuando ocurre un error.
Es adecuado para **procesos que pueden tener éxito si se reintentan después del fallo**, como fallos de red temporales.

## 🔰 Sintaxis Básica y Operación

### retry(count) - Forma Básica

```ts
import { throwError, of } from 'rxjs';
import { retry, catchError } from 'rxjs';

throwError(() => new Error('Error temporal'))
  .pipe(
    retry(2), // Reintentar hasta 2 veces
    catchError((error) => of(`Error final: ${error.message}`))
  )
  .subscribe(console.log);
// Salida:
// Error final: Error temporal
```

En este ejemplo, se realizan hasta dos reintentos después del primer fallo, y se emite un mensaje en respaldo si todos fallan.

### retry(config) - Formato de Objeto de Configuración (RxJS 7.4+)

En RxJS 7.4 y posteriores, es posible un control más detallado pasando un objeto de configuración.

```ts
import { throwError, of } from 'rxjs';
import { retry, catchError, tap } from 'rxjs';

let attemptCount = 0;

throwError(() => new Error('Error temporal'))
  .pipe(
    tap({
      subscribe: () => {
        attemptCount++;
        console.log(`Intento ${attemptCount}`);
      }
    }),
    retry({
      count: 2,           // Reintentar hasta 2 veces
      delay: 1000,        // Esperar 1 segundo antes de reintentar (usa asyncScheduler internamente)
      resetOnSuccess: true // Restablecer contador en caso de éxito
    }),
    catchError((error) => of(`Error final: ${error.message}`))
  )
  .subscribe(console.log);

// Salida:
// Intento 1
// Intento 2
// Intento 3
// Error final: Error temporal
```

> [!NOTE] Control de Temporización de Reintento
> Cuando se especifica la opción `delay`, se usa **asyncScheduler** internamente. Para un control de temporización de reintento más detallado (retroceso exponencial, etc.), consulte [Tipos de Scheduler y Uso - Control de Reintento de Error](/es/guide/schedulers/types#error-retry-control).

[🌐 Documentación Oficial de RxJS - retry](https://rxjs.dev/api/index/function/retry)

## 💡 Ejemplo de Uso Típico

El siguiente ejemplo es una configuración que reintenta **procesamiento asíncrono con éxito/fallo aleatorio** hasta 3 veces.

```ts
import { interval, throwError, of } from 'rxjs';
import { mergeMap, retry, catchError } from 'rxjs';

let attempt = 0;

interval(1000)
  .pipe(
    mergeMap(() => {
      attempt++;
      const shouldFail = Math.random() < 0.8;

      if (shouldFail) {
        return throwError(() => new Error(`Fallo #${attempt}`));
      } else {
        return of(`Éxito #${attempt}`);
      }
    }),
    retry(3),
    catchError((err) => of(`Fallo final: ${err.message}`))
  )
  .subscribe(console.log);
// Salida:
// Éxito #1
// Éxito #5
// Éxito #6
// Fallo final: Fallo #7
```

## 🧪 Ejemplo de Código Práctico (con UI)

```ts
import { interval, throwError, of } from 'rxjs';
import { mergeMap, retry, catchError } from 'rxjs';

// Área de visualización de salida
const retryOutput = document.createElement('div');
retryOutput.innerHTML = '<h3>Ejemplo de retry (Simulación de Solicitud API):</h3>';
document.body.appendChild(retryOutput);

// Visualización de estado de solicitud
const requestStatus = document.createElement('div');
requestStatus.style.marginTop = '10px';
requestStatus.style.padding = '10px';
requestStatus.style.border = '1px solid #ddd';
requestStatus.style.maxHeight = '200px';
requestStatus.style.overflowY = 'auto';
retryOutput.appendChild(requestStatus);

// Solicitud de API que tiene éxito o falla aleatoriamente
let attemptCount = 0;

function simulateRequest() {
  attemptCount++;

  const logEntry = document.createElement('div');
  logEntry.textContent = `Intento #${attemptCount} Enviando solicitud...`;
  requestStatus.appendChild(logEntry);

  return interval(1000).pipe(
    mergeMap(() => {
      const shouldFail = Math.random() < 0.8;

      if (shouldFail) {
        const errorMsg = document.createElement('div');
        errorMsg.textContent = `Intento #${attemptCount} Fallido: Error de red`;
        errorMsg.style.color = 'red';
        requestStatus.appendChild(errorMsg);

        return throwError(() => new Error('Error de red'));
      } else {
        const successMsg = document.createElement('div');
        successMsg.textContent = `Intento #${attemptCount} ¡Éxito!`;
        successMsg.style.color = 'green';
        requestStatus.appendChild(successMsg);

        return of({ id: 1, name: 'Datos recuperados exitosamente' });
      }
    }),
    retry(3),
    catchError((err) => {
      const finalError = document.createElement('div');
      finalError.textContent = `Todos los reintentos fallaron: ${err.message}`;
      finalError.style.color = 'red';
      finalError.style.fontWeight = 'bold';
      requestStatus.appendChild(finalError);

      return of({ error: true, message: 'Reintento fallido' });
    })
  );
}

// Botón de inicio de solicitud
const startButton = document.createElement('button');
startButton.textContent = 'Iniciar Solicitud';
startButton.style.padding = '8px 16px';
startButton.style.marginTop = '10px';
retryOutput.insertBefore(startButton, requestStatus);

startButton.addEventListener('click', () => {
  attemptCount = 0;
  requestStatus.innerHTML = '';
  startButton.disabled = true;

  simulateRequest().subscribe((result) => {
    const resultElement = document.createElement('div');
    if ('error' in result) {
      resultElement.textContent = `Resultado final: ${result.message}`;
      resultElement.style.backgroundColor = '#ffebee';
    } else {
      resultElement.textContent = `Resultado final: ${result.name}`;
      resultElement.style.backgroundColor = '#e8f5e9';
    }

    resultElement.style.padding = '10px';
    resultElement.style.marginTop = '10px';
    resultElement.style.borderRadius = '5px';
    requestStatus.appendChild(resultElement);

    startButton.disabled = false;
  });
});
```

## ✅ Resumen

- `retry(n)` reintenta hasta `n` veces si Observable falla
- `retry` se **reintenta hasta que se completa exitosamente** (el fallo continuo resulta en un error)
- Útil para **APIs asíncronas y solicitudes de red** donde ocurren fallos temporales
- Comúnmente combinado con `catchError` para especificar **procesamiento de respaldo**
- A partir de RxJS 7.4+, es posible especificar `delay`, `resetOnSuccess`, etc. en formato de objeto de configuración

## Páginas Relacionadas

- [retry y catchError](/es/guide/error-handling/retry-catch) - Patrones para combinar retry y catchError, ejemplos de uso práctico
- [Depuración de Reintento](/es/guide/error-handling/retry-catch#retry-debugging) - Cómo rastrear el contador de intentos (5 patrones de implementación)
- [Tipos de Scheduler y Uso](/es/guide/schedulers/types#error-retry-control) - Control de temporización de reintento detallado, implementación de retroceso exponencial
- [Técnicas de Depuración de RxJS](/es/guide/debugging/#scenario-6-track-retry-attempt-count) - Escenarios de depuración de reintento
