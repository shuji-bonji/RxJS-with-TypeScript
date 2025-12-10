---
description: Se explicarán estrategias robustas de manejo de errores que combinan los operadores retry y catchError. Aprenderá a través de ejemplos prácticos de solicitudes de API, como reintentos para fallos temporales, patrones de backoff exponencial y manejo adecuado de fallback.
titleTemplate: ':title | RxJS'
---
# retry y catchError - Combinación Efectiva de Manejo de Errores

Los dos operadores en el corazón del manejo de errores en RxJS, `retry` y `catchError`, se describen en detalle. Juntos, proporcionan una estrategia robusta de manejo de errores.

## retry - Reintentar en Caso de Fallo (Patrón Básico)

El operador `retry` se utiliza para **reanudar la ejecución de un flujo un número especificado de veces** cuando ocurre un error en el flujo. Esto es especialmente útil para operaciones que pueden fallar temporalmente, como solicitudes de red.

[🌐 Documentación Oficial de RxJS - retry](https://rxjs.dev/api/index/function/retry)

### Patrón Básico

```ts
import { Observable, of } from 'rxjs';
import { retry, map } from 'rxjs';

// Función que genera errores aleatoriamente
function getDataWithRandomError(): Observable<string> {
  return of('data').pipe(
    map(() => {
      if (Math.random() < 0.7) {
        throw new Error('Ocurrió un error aleatorio');
      }
      return '¡Obtención de datos exitosa!';
    })
  );
}

// Reintentar hasta 3 veces
getDataWithRandomError()
  .pipe(retry(3))
  .subscribe({
    next: (data) => console.log('Éxito:', data),
    error: (err) => console.error('Error (después de 3 reintentos):', err.message),
  });

// Salida:
// Éxito: ¡Obtención de datos exitosa!
// Error (después de 3 reintentos): Ocurrió un error aleatorio ⇦ Mostrado cuando fallan los 3 reintentos
```

### Monitoreo en Tiempo Real del Estado de Reintento

```ts
import { Observable, of } from 'rxjs';
import { retry, tap, catchError, map } from 'rxjs';

let attempts = 0;

function simulateFlakyRequest(): Observable<string> {
  return of('request').pipe(
    tap(() => {
      attempts++;
      console.log(`Intento #${attempts}`);
    }),
    map(() => {
      if (attempts < 3) {
        throw new Error(`Error #${attempts}`);
      }
      return '¡Éxito!';
    })
  );
}

simulateFlakyRequest()
  .pipe(
    retry(3),
    catchError((error) => {
      console.log('Fallaron todos los reintentos:', error.message);
      return of('Valor de fallback');
    })
  )
  .subscribe({
    next: (result) => console.log('Resultado final:', result),
    complete: () => console.log('Completado'),
  });

// Salida:
// Intento #1
// Intento #2
// Intento #3
// Resultado final: ¡Éxito!
// Completado
```

> [!NOTE] Temporización de Reintentos y Schedulers
> Cuando se especifica un tiempo de retraso en el operador `retry` (como `retry({ delay: 1000 })`), **asyncScheduler** se usa internamente. Al utilizar schedulers, puede controlar el momento de los reintentos en detalle y usar tiempo virtual durante las pruebas.
>
> Para más información, consulte [Tipos de Scheduler y Uso - Control de Reintentos de Errores](/es/guide/schedulers/types#error-retry-control).

## catchError - Captura de Errores y Manejo Alternativo (Patrón Básico)

El operador `catchError` captura errores que ocurren en el flujo y los maneja **devolviendo un Observable alternativo**. Esto permite que el procesamiento continúe sin interrumpir el flujo cuando ocurre un error.

[🌐 Documentación Oficial de RxJS - catchError](https://rxjs.dev/api/index/function/catchError)

### Patrón Básico

```ts
import { of, throwError } from 'rxjs';
import { catchError } from 'rxjs';

throwError(() => new Error('Error en la llamada de API')) // RxJS 7+, forma funcional recomendada
  .pipe(
    catchError((error) => {
      console.error('Ocurrió un error:', error.message);
      return of('Valor predeterminado en caso de error');
    })
  )
  .subscribe({
    next: (value) => console.log('Valor:', value),
    complete: () => console.log('Completado'),
  });

// Salida:
// Ocurrió un error: Error en la llamada de API
// Valor: Valor predeterminado en caso de error
// Completado
```

### Relanzar Errores

Si desea relanzar un error después de que se ha registrado

```ts
import { throwError } from 'rxjs';
import { catchError } from 'rxjs';

throwError(() => new Error('Error original')) // RxJS 7+, forma funcional recomendada
  .pipe(
    catchError((error) => {
      console.error('Registrando error:', error.message);
      // Relanzar error
      return throwError(() => new Error('Error convertido'));
    })
  )
  .subscribe({
    next: (value) => console.log('Valor:', value),
    error: (err) => console.error('Error final:', err.message),
    complete: () => console.log('Completado'),
  });

// Salida:
// Registrando error: Error original
// Error final: Error convertido
```

## Combinación de retry y catchError

En aplicaciones reales, es común usar una combinación de `retry` y `catchError`. Esta combinación permite que los errores temporales se resuelvan mediante reintentos, mientras se proporciona un valor de fallback en caso de fallo eventual.

```ts
import { of, throwError } from 'rxjs';
import { retry, catchError, tap } from 'rxjs';

function fetchData() {
  // Observable que genera un error
  return throwError(() => new Error('Error de red')) // RxJS 7+, forma funcional recomendada
    .pipe(
    // Para depuración
    tap(() => console.log('Intentando obtener datos')),
    // Reintentar hasta 3 veces
    retry(3),
    // Si fallan todos los reintentos
    catchError((error) => {
      console.error('Fallaron todos los reintentos:', error.message);
      // Retornar valor predeterminado
      return of({
        error: true,
        data: null,
        message: 'Falló la obtención de datos',
      });
    })
  );
}

fetchData().subscribe({
  next: (result) => console.log('Resultado:', result),
  complete: () => console.log('Procesamiento completado'),
});

// Salida:
// Fallaron todos los reintentos: Error de red
// Resultado: {error: true, data: null, message: 'Falló la obtención de datos'}
// Procesamiento completado
```

## Estrategia Avanzada de Reintento: retryWhen

Si necesita una estrategia de reintento más flexible, puede usar el operador `retryWhen`. Esto le permite personalizar el momento y la lógica de reintento.


[🌐 Documentación Oficial de RxJS - retryWhen](https://rxjs.dev/api/index/function/retryWhen)

### Reintentar con Backoff Exponencial

El patrón de backoff exponencial (aumento gradual de los intervalos de reintento) es común para reintentos de solicitudes de red. Esto reduce la carga en el servidor mientras se espera que se resuelvan los problemas temporales.

```ts
import { throwError, timer, of } from 'rxjs';
import { retryWhen, tap, concatMap, catchError } from 'rxjs';

function fetchWithRetry() {
  let retryCount = 0;

  return throwError(() => new Error('Error de red')).pipe(
    retryWhen((errors) =>
      errors.pipe(
        // Contar ocurrencias de errores
        tap((error) => console.log('Ocurrió un error:', error.message)),
        // Retrasar con backoff exponencial
        concatMap(() => {
          retryCount++;
          const delayMs = Math.min(1000 * Math.pow(2, retryCount), 10000);
          console.log(`Intento de reintento ${retryCount} después de ${delayMs}ms`);
          // timer usa asyncScheduler internamente
          return timer(delayMs);
        }),
        // Reintentar hasta 5 veces
        tap(() => {
          if (retryCount >= 5) {
            throw new Error('Se excedió el máximo de intentos de reintento');
          }
        })
      )
    ),
    // Fallback final
    catchError((error) => {
      console.error('Fallaron todos los reintentos:', error.message);
      return of({
        error: true,
        message: 'Conexión fallida. Por favor, inténtelo de nuevo más tarde.',
      });
    })
  );
}

fetchWithRetry().subscribe({
  next: (result) => console.log('Resultado:', result),
  error: (err) => console.error('Error no manejado:', err),
});

// Salida:
// Ocurrió un error: Error de red
// Intento de reintento 1 después de 2000ms
// Ocurrió un error: Error de red
// Intento de reintento 2 después de 4000ms
// Ocurrió un error: Error de red
// Intento de reintento 3 después de 8000ms
```

> [!TIP] Control Detallado de Reintentos Usando Schedulers
> En el ejemplo anterior, se usa `timer()`, pero si se necesita un control más avanzado, puede especificar un scheduler explícitamente para afinar el momento de los reintentos o usar tiempo virtual durante las pruebas.
>
> Para más información, consulte [Tipos de Scheduler y Uso - Control de Reintentos de Errores](/es/guide/schedulers/types#error-retry-control).

## Depuración de Reintentos

Al depurar el proceso de reintento, es importante realizar un seguimiento del número de intentos y los resultados de cada intento. A continuación se presentan algunas formas prácticas de monitorear el estado de reintento en tiempo real.

### Método 1: Callback error de tap (Básico)

El callback `error` del operador `tap` se puede usar para contar el número de intentos cuando ocurre un error.

```typescript
import { throwError, of, retry, catchError, tap } from 'rxjs';
let attemptCount = 0;

throwError(() => new Error('Error temporal'))
  .pipe(
    tap({
      error: () => {
        attemptCount++;
        console.log(`Conteo de intentos: ${attemptCount}`);
      }
    }),
    retry(2),
    catchError((error) => {
      console.log(`Conteo final de intentos: ${attemptCount}`);
      return of(`Error final: ${error.message}`);
    })
  )
  .subscribe({
    next: console.log,
    error: err => console.error('Error de suscripción:', err)
  });

// Salida:
// Conteo de intentos: 1
// Conteo de intentos: 2
// Conteo de intentos: 3
// Conteo final de intentos: 3
// Error final: Error temporal
```

> [!NOTE] Limitaciones con throwError
> `throwError` emite un error inmediatamente sin emitir un valor, por lo que el callback `next` de `tap` no se ejecuta. Se debe usar el callback `error`.

### Método 2: Seguimiento Detallado con retryWhen (Recomendado)

Para rastrear información más detallada (número de intentos, tiempo de retraso, detalles de error), use `retryWhen`.

```typescript
import { throwError, of, timer, retryWhen, mergeMap, catchError } from 'rxjs';
throwError(() => new Error('Error temporal'))
  .pipe(
    retryWhen((errors) =>
      errors.pipe(
        mergeMap((error, index) => {
          const retryCount = index + 1;
          console.log(`━━━━━━━━━━━━━━━━━━━━━━━━━━━`);
          console.log(`🔄 Reintento ${retryCount}`);
          console.log(`   Error: ${error.message}`);

          if (retryCount > 2) {
            console.log(`❌ Se alcanzó el conteo máximo de reintentos`);
            console.log(`━━━━━━━━━━━━━━━━━━━━━━━━━━━`);
            throw error;
          }

          const delayMs = 1000;
          console.log(`⏳ Reintentando después de ${delayMs}ms...`);
          console.log(`━━━━━━━━━━━━━━━━━━━━━━━━━━━`);

          return timer(delayMs);
        })
      )
    ),
    catchError((error) => {
      console.log(`\nResultado final: Fallaron todos los reintentos`);
      return of(`Error final: ${error.message}`);
    })
  )
  .subscribe(result => console.log('Resultado:', result));

// Salida:
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━
// 🔄 Reintento 1
//    Error: Error temporal
// ⏳ Reintentando después de 1000ms...
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━
// (Esperar 1 segundo)
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━
// 🔄 Reintento 2
//    Error: Error temporal
// ⏳ Reintentando después de 1000ms...
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━
// (Esperar 1 segundo)
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━
// 🔄 Reintento 3
//    Error: Error temporal
// ❌ Se alcanzó el conteo máximo de reintentos
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━
//
// Resultado final: Fallaron todos los reintentos
// Resultado: Error final: Error temporal
```

### Método 3: Rastrear Intentos con un Observable Personalizado

Para Observables que emiten valores, como solicitudes de API reales, puede gestionar el número de intentos con un Observable personalizado.

```typescript
import { Observable, of, retry, catchError } from 'rxjs';
let attemptCount = 0;

// Observable que puede contar intentos
const retryableStream$ = new Observable(subscriber => {
  attemptCount++;
  console.log(`[Intento ${attemptCount}]`);

  // Los primeros 2 intentos fallan, el 3er intento tiene éxito
  if (attemptCount < 3) {
    subscriber.error(new Error(`Falló (intento ${attemptCount})`));
  } else {
    subscriber.next('Datos de éxito');
    subscriber.complete();
  }
});

retryableStream$
  .pipe(
    retry(2),
    catchError((error) => {
      console.log(`[Completado] Falló después de un total de ${attemptCount} intentos`);
      return of(`Error final: ${error.message}`);
    })
  )
  .subscribe({
    next: data => console.log('[Resultado]', data),
    complete: () => console.log('[Completado]')
  });

// Salida:
// [Intento 1]
// [Intento 2]
// [Intento 3]
// [Resultado] Datos de éxito
// [Completado]
```

### Método 4: Backoff Exponencial y Registro

Este es un patrón de registro detallado para solicitudes de API prácticas.

```typescript
import { timer, throwError, of, retryWhen, mergeMap, catchError, finalize } from 'rxjs';
import { ajax } from 'rxjs/ajax';

function fetchWithRetryLogging(url: string, maxRetries = 3) {
  let startTime = Date.now();

  return ajax.getJSON(url).pipe(
    retryWhen((errors) =>
      errors.pipe(
        mergeMap((error, index) => {
          const retryCount = index + 1;
          const elapsed = Date.now() - startTime;

          console.log(`━━━━━━━━━━━━━━━━━━━━━━━━━━━`);
          console.log(`🔄 Información de Reintento`);
          console.log(`   Conteo: ${retryCount}/${maxRetries}`);
          console.log(`   Error: ${error.message || error.status}`);
          console.log(`   Tiempo transcurrido: ${elapsed}ms`);

          if (retryCount >= maxRetries) {
            console.log(`❌ Se alcanzó el número máximo de reintentos`);
            console.log(`━━━━━━━━━━━━━━━━━━━━━━━━━━━`);
            throw error;
          }

          // Backoff exponencial
          const delayMs = Math.min(1000 * Math.pow(2, index), 10000);
          console.log(`⏳ Reintentando después de ${delayMs}ms...`);
          console.log(`━━━━━━━━━━━━━━━━━━━━━━━━━━━`);

          return timer(delayMs);
        })
      )
    ),
    catchError((error) => {
      const totalTime = Date.now() - startTime;
      console.log(`\n❌ Fallo final (tiempo total: ${totalTime}ms)`);
      return of({ error: true, message: 'Falló la adquisición de datos' });
    }),
    finalize(() => {
      const totalTime = Date.now() - startTime;
      console.log(`\n✅ Procesamiento completado (tiempo total: ${totalTime}ms)`);
    })
  );
}

// Ejemplo de uso
fetchWithRetryLogging('https://jsonplaceholder.typicode.com/users/1').subscribe({
  next: data => console.log('Datos:', data),
  error: err => console.error('Error:', err)
});
```

### Método 5: Objeto de Configuración de retry en RxJS 7.4+

En RxJS 7.4+ y posteriores, puede pasar un objeto de configuración a `retry`.

```typescript
import { throwError, of, retry, catchError, tap } from 'rxjs';
let attemptCount = 0;

throwError(() => new Error('Error temporal'))
  .pipe(
    tap({
      subscribe: () => {
        attemptCount++;
        console.log(`Intento ${attemptCount}`);
      },
      error: (err) => console.log(`Ocurrió un error:`, err.message)
    }),
    retry({
      count: 2,
      delay: 1000, // Esperar 1 segundo antes del reintento (usa asyncScheduler internamente)
      resetOnSuccess: true
    }),
    catchError((error) => {
      console.log(`Fallo final (total de ${attemptCount} intentos)`);
      return of(`Error final: ${error.message}`);
    })
  )
  .subscribe(result => console.log('Resultado:', result));

// Salida:
// Intento 1
// Ocurrió un error: Error temporal
// Intento 2
// Ocurrió un error: Error temporal
// Intento 3
// Ocurrió un error: Error temporal
// Fallo final (total de 3 intentos)
// Resultado: Error final: Error temporal
```

> [!TIP] Enfoque Recomendado para la Depuración de Reintentos
> - **Durante el desarrollo**: Método 2 (retryWhen) o Método 4 (registro detallado) es óptimo
> - **Entorno de producción**: Basado en el Método 4, agregue envío de logs al servicio de monitoreo de errores
> - **Casos simples**: Método 1 (tap error) o Método 5 (config de retry) es suficiente
>
> **Información Relacionada**:
> - Para el control del momento de reintento, consulte [Tipos de Scheduler y Uso - Control de Reintentos de Errores](/es/guide/schedulers/types#error-retry-control)
> - Para la visión general de las técnicas de depuración, consulte [Técnicas de Depuración de RxJS - Rastreo de Intentos de Reintento](/es/guide/debugging/#scenario-6-tracking-retry-attempts)

## Ejemplo de Uso en una Aplicación Real: Solicitud de API

Aquí hay un ejemplo de utilización de estos operadores en una solicitud de API real.

```ts
import { Observable, of } from 'rxjs';
import { ajax } from 'rxjs/ajax';
import { retry, catchError, finalize, tap } from 'rxjs';

// Estado de carga
let isLoading = false;

function fetchUserData(userId: string): Observable<any> {
  isLoading = true;

  return ajax.getJSON(`https://jsonplaceholder.typicode.com/users/${userId}`).pipe(
    // Depuración de solicitud
    tap((response) => console.log('Respuesta de API:', response)),
    // Reintentar errores de red hasta 2 veces
    retry(2),
    // Manejo de errores
    catchError((error) => {
      if (error.status === 404) {
        return of({ error: true, message: 'Usuario no encontrado' });
      } else if (error.status >= 500) {
        return of({ error: true, message: 'Ha ocurrido un error en el servidor' });
      }
      return of({ error: true, message: 'Ha ocurrido un error desconocido' });
    }),
    // Siempre ejecutar independientemente del éxito o fracaso
    finalize(() => {
      isLoading = false;
      console.log('Carga completada');
    })
  );
}

// Ejemplo de uso
fetchUserData('123').subscribe({
  next: (data) => {
    if (data.error) {
      // Mostrar información de error
      console.error('Error:', data.message);
    } else {
      // Mostrar datos
      console.log('Datos de usuario:', data);
    }
  },
});


// Salida:
// GET https://jsonplaceholder.typicode.com/users/123 net::ERR_NAME_NOT_RESOLVED
// GET https://jsonplaceholder.typicode.com/users/123 net::ERR_NAME_NOT_RESOLVED
// Ha ocurrido un error desconocido
// Carga completada
// GET https://jsonplaceholder.typicode.com/users/123 net::ERR_NAME_NOT_RESOLVED
```

## Mejores Prácticas

### ¿Cuándo Debo Usar retry?

- **Cuando se esperan errores temporales** (por ejemplo, problemas de conexión de red)
- **Problemas temporales** en el lado del servidor (por ejemplo, alta carga, tiempos de espera)
- Para errores que **pueden** resolverse mediante reintento

### ¿Cuándo NO Debe Usarse retry?

- **Error de autenticación** (401, 403) - el reintento no lo resolverá
- **El recurso no existe** (404) - el reintento no lo encuentra
- **Error de validación** (400) - hay un problema con la solicitud en sí
- **Error de programa del lado del cliente** - el reintento es inútil

### Uso Efectivo de catchError

- Manejar **de manera diferente** según el **tipo** de error
- Proporcionar **mensaje claro** al usuario
- Retornar **datos de fallback** cuando sea apropiado
- Convertir errores según sea necesario

## Resumen

La combinación de `retry` y `catchError` proporciona un manejo robusto de errores. Los errores temporales se pueden recuperar mediante reintentos, y los errores persistentes se pueden manejar apropiadamente con fallback para mejorar la experiencia del usuario. En aplicaciones del mundo real, es importante seleccionar la estrategia apropiada y proporcionar un mecanismo de fallback dependiendo de la naturaleza del error.

Las siguientes secciones describen el operador `finalize` para la liberación de recursos y el proceso de finalización del flujo.
