---
description: "Describe una estrategia completa de manejo de errores para RxJS, incluyendo cómo combinar los operadores catchError, retry, retryWhen y finalize, retry con backoff exponencial, clasificación de errores y manejo apropiado, manejadores de errores globales y otras estrategias robustas de manejo de errores en TypeScript. Cómo implementar un manejo robusto de errores en TypeScript."
---

# Estrategia de gestión de errores de RxJS

El manejo de errores en RxJS es un aspecto importante de la programación reactiva. Implementar un adecuado manejo de errores mejora la robustez y confiabilidad de tu aplicación. Este documento describe las diferentes estrategias de manejo de errores disponibles en RxJS.

## Patrones básicos

RxJS maneja los errores como parte del ciclo de vida del Observable. El manejo básico de errores incluye los siguientes métodos.

```ts
import { of, throwError } from 'rxjs';
import { catchError } from 'rxjs';

// Error.Observable
const error$ = throwError(() => new Error('Se ha producido un error.')); // RxJS 7Formato de función recomendado a partir de ahora

// Tratamiento básico de errores
error$
  .pipe(
    catchError((error: unknown) => {
      const message = error instanceof Error ? error.message : String(error);
      console.error('Capturar error:', message);
      return of('Valor de retorno tras error');
    })
  )
  .subscribe({
    next: (value) => console.log('Valor:', value),
    error: (err) => console.error('Errores no gestionados:', err),
    complete: () => console.log('Completado'),
  });

// Salida:
// Capturar error: Se ha producido un error.
// Valor: Valor de retorno tras error
// Completado
```

## Varias estrategias de manejo de errores

### 1. Capturar errores y proporcionar valores alternativos

Utilice el operador `catchError` para capturar errores y proporcionar valores alternativos o flujos alternativos.

```ts
import { of, throwError } from 'rxjs';
import { catchError } from 'rxjs';

const source$ = throwError(() => new Error('Error de adquisición de datos'));

source$.pipe(
  catchError((error: unknown) => {
    const message = error instanceof Error ? error.message : String(error);
    console.error('Se ha producido un error:', message);
    // Devolver datos alternativos
    return of({ isError: true, data: [], message: 'Mostrar datos por defecto' });
  })
).subscribe(data => console.log('Resultado:', data));

// Salida:
// Se ha producido un error: Error de adquisición de datos
// Resultado: {isError: true, data: Array(0), message: 'Mostrar datos por defecto'}
```

### 2. Reintentar si se produce un error

Utilice el operador `retry` para reintentar el flujo si se produce un error (a partir de la versión 7.3, se recomienda el formato `retry({ count, delay })`, que sustituye al antiguo `retryWhen`).

```ts
import { interval, throwError, of } from 'rxjs';
import { mergeMap, retry, tap } from 'rxjs';

let attemptCount = 0;

interval(1000).pipe(
  mergeMap(val => {
    if (++attemptCount <= 2) {
      return throwError(() => new Error(`Error #${attemptCount}`));
    }
    return of('Éxito！');
  }),
  tap(() => console.log('Ejecución:', attemptCount)),
  retry(2), // Máx.2Reintentos
).subscribe({
  next: value => console.log('Valor:', value),
  error: err => console.error('Error final:', err.message),
});

// Salida:
// Ejecución: 3
// Valor: Éxito！
// Ejecución: 4
// Valor: Éxito！
// Ejecución: 5
// ...
```

### 3. retry with exponential backoff

El backoff exponencial, que aumenta gradualmente el intervalo de reintento, es eficaz, por ejemplo, para las solicitudes de red.

```ts
import { throwError, timer, of } from 'rxjs';
import { retry, tap, catchError } from 'rxjs';

function fetchWithRetry() {
  return throwError(() => new Error('Error de red')).pipe(
    // RxJS 7.3+ Recomendado: retry({ count, delay }) Formato
    retry({
      count: 5, // Máx.5Hasta tres reintentos
      delay: (error: unknown, retryCount) => {
        const message = error instanceof Error ? error.message : String(error);
        console.log('Se ha producido un error:', message);
        // Retroceso exponencial (máx.10segundos)
        const delayMs = Math.min(1000 * Math.pow(2, retryCount), 10000);
        console.log(`${retryCount}Reintento por segunda vez${delayMs}msEjecutado después de`);
        return timer(delayMs);
      }
    }),
    // Reintento final
    catchError((error: unknown) => {
      const message = error instanceof Error ? error.message : String(error);
      console.error('Todos los reintentos fallidos:', message);
      return of({
        error: true,
        message: 'Conexión fallida. Vuelva a intentarlo más tarde.',
      });
    })
  );
}

fetchWithRetry().subscribe({
  next: (result) => console.log('Resultado:', result),
  error: (err) => console.error('Errores no gestionados:', err),
});

// Salida:
// Se ha producido un error: Error de red
// 1Reintento por segunda vez2000msEjecutado después de
// Se ha producido un error: Error de red
// 2Reintento por segunda vez4000msEjecutado después de
// Se ha producido un error: Error de red
// 3Reintento por segunda vez8000msEjecutado después de
// Se ha producido un error: Error de red
// 4Reintento por segunda vez10000msEjecutado después de
// Se ha producido un error: Error de red
// 5Reintento por segunda vez10000msEjecutado después de
// Todos los reintentos fallidos: Se ha superado el número máximo de reintentos
// Resultado: {error: true, message: 'Conexión fallida. Vuelva a intentarlo más tarde.'}
```

### 4. Liberación de recursos en caso de error

Utilice el operador `finalize` para liberar recursos cuando un flujo termina **completo o por error**.
Finalize es útil cuando desea asegurarse de que el proceso de limpieza se realiza no sólo en caso de error, sino también en caso de finalización normal.

```ts
import { throwError } from 'rxjs';
import { catchError, finalize } from 'rxjs';

let isLoading = true;

throwError(() => new Error('Error de procesamiento'))
  .pipe(
    catchError((error: unknown) => {
      const message = error instanceof Error ? error.message : String(error);
      console.error('Error de procesamiento:', message);
      return throwError(() => error); // Reintentar error
    }),
    finalize(() => {
      isLoading = false;
      console.log('Restablecer estado de carga:', isLoading);
    })
  )
  .subscribe({
    next: (value) => console.log('Valor:', value),
    error: (err) => console.error('Error final:', err.message),
    complete: () => console.log('Completado'),
  });

// Salida:
// Error de procesamiento: Error de procesamiento
// Error final: Error de procesamiento
// Restablecer estado de carga: false
```

## Patrón de manejo de errores

### Tratamiento de errores incluyendo el control de la visualización de los elementos de la interfaz de usuario

```ts
import { of, throwError } from 'rxjs';
import { catchError, finalize, tap } from 'rxjs';

function fetchData(shouldFail = false) {
  // Visualización de carga
  showLoadingIndicator();

  // Adquisición de datos (éxito o error)
  return (
    shouldFail
      ? throwError(() => new Error('APIError'))
      : of({ name: 'Datos', value: 42 })
  ).pipe(
    tap((data) => {
      // Con éxito
      updateUI(data);
    }),
    catchError((error: unknown) => {
      const message = error instanceof Error ? error.message : String(error);
      // En caso de errorUIActualización
      showErrorMessage(message);
      // Devuelve datos vacíos o valor por defecto
      return of({ name: 'Por defecto', value: 0 });
    }),
    finalize(() => {
      // Silenciar la pantalla de carga independientemente del éxito o error
      hideLoadingIndicator();
    })
  );
}

// UIFunciones auxiliares para operaciones
function showLoadingIndicator() {
  console.log('Visualización de carga');
}
function hideLoadingIndicator() {
  console.log('Ocultar carga');
}
function updateUI(data: { name: string; value: number }) {
  console.log('UIActualización:', data);
}
function showErrorMessage(message: any) {
  console.log('Visualización de errores:', message);
}

// Ejemplo de uso
fetchData(true).subscribe();

// Salida:
// Visualización de carga
// Visualización de errores: APIError
// Ocultar carga
```

### Manejo de múltiples fuentes de error

```ts
import { forkJoin, of, throwError } from 'rxjs';
import { catchError, map } from 'rxjs';

// Simular variasAPISimular solicitudes
function getUser() {
  return of({ id: 1, name: 'Taro Yamada' });
}

function getPosts() {
  return throwError(() => new Error('Error de adquisición posterior'));
}

function getComments() {
  return throwError(() => new Error('Comentar error de adquisición'));
}

// Recuperar todos los datos y permitir errores parciales
forkJoin({
  user: getUser().pipe(
    catchError((error: unknown) => {
      const message = error instanceof Error ? error.message : String(error);
      console.error('Error de adquisición del usuario:', message);
      return of(null); // En caso de errornullDevuelve un array vacío
    })
  ),
  posts: getPosts().pipe(
    catchError((error: unknown) => {
      const message = error instanceof Error ? error.message : String(error);
      console.error('Error de adquisición posterior:', message);
      return of([]); // Se devuelve un array vacío en caso de error
    })
  ),
  comments: getComments().pipe(
    catchError((error: unknown) => {
      const message = error instanceof Error ? error.message : String(error);
      console.error('Comentar error de adquisición:', message);
      return of([]); // Se devuelve un array vacío en caso de error
    })
  ),
})
  .pipe(
    map((result) => ({
      ...result,
      // Añade una bandera para indicar si hubo un error parcial
      hasErrors:
        !result.user ||
        result.posts.length === 0 ||
        result.comments.length === 0,
    }))
  )
  .subscribe((data) => {
    console.log('Resultado final:', data);

    if (data.hasErrors) {
      console.log(
        'Error al recuperar algunos datos, pero muestra los datos disponibles'
      );
    }
  });

// Salida:
// Error de adquisición posterior: Error de adquisición posterior
// Comentar error de adquisición: Comentar error de adquisición
// Resultado final: {user: {…}, posts: Array(0), comments: Array(0), hasErrors: true}
// Error al recuperar algunos datos, pero muestra los datos disponibles
```

## Mejores prácticas para la gestión de errores

1. **Siempre captura errores**: añade siempre la gestión de errores en la cadena Observable. Esto es especialmente importante para flujos de larga duración.

2.**Proporcionar mensajes de error significativos**: incluir información en el objeto de error para ayudar a determinar dónde ocurrió y qué lo causó.

3, **liberar recursos correctamente**: utiliza `finalize` para asegurarte de que los recursos se liberan independientemente del éxito o el fracaso.

4.**Considera estrategias de reintento**: especialmente para operaciones de red, implementar una estrategia de reintento adecuada mejora la fiabilidad.

5. **Manejo de errores amigable**: en la interfaz de usuario, proporciona información que los usuarios puedan entender, en lugar de mostrar mensajes de error técnicos tal cual.

```ts
// Ejemplo.：Conversión a mensajes de error fáciles de usar
function getErrorMessage(error: any): string {
  if (error.status === 401) {
    return 'La sesión ha expirado. Vuelva a iniciar sesión.';
  } else if (error.status === 404) {
    return 'No se ha podido encontrar el recurso solicitado.';
  } else if (error.status >= 500) {
    return 'Se ha producido un error en el servidor. Vuelva a intentarlo más tarde.';
  }
  return 'Se ha producido un error inesperado.';
}
```

## Resumen.

El manejo de errores en RxJS es una parte importante para asegurar la robustez de la aplicación. Usando la combinación correcta de operadores como `catchError`, `retry` y `finalize`, se pueden manejar una gran variedad de escenarios de error. Diseña una estrategia integral de gestión de errores para mejorar la experiencia del usuario, en lugar de limitarte a capturar errores.

## 🔗 Secciones relacionadas.

- **[Errores comunes y cómo tratarlos](/es/guide/anti-patterns/common-mistakes#9-error-grasping)** - Revisa los anti-patrones sobre el manejo de errores.
- **[retry y catchError](/es/guide/error-handling/retry-catch)** - Explica más detalladamente su utilización.