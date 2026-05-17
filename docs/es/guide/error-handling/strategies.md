---
description: Se describirá una estrategia integral de manejo de errores en RxJS, incluyendo cómo combinar los operadores catchError, retry, retryWhen y finalize, reintentos con backoff exponencial, liberación de recursos en caso de error, manejo de fallback y otros patrones prácticos.
---
# Estrategias de Manejo de Errores en RxJS

El manejo de errores en RxJS es un aspecto importante de la programación reactiva. Implementar un manejo de errores adecuado mejora la robustez y confiabilidad de su aplicación. Este documento describe las diversas estrategias de manejo de errores disponibles en RxJS.

## Patrón Básico

RxJS maneja errores como parte del ciclo de vida del Observable. El manejo básico de errores incluye los siguientes métodos:

```ts
import { of, throwError } from 'rxjs';
import { catchError } from 'rxjs';

// Observable que genera un error
const error$ = throwError(() => new Error('Ocurrió un error')); // RxJS 7+, forma funcional recomendada

// Manejo básico de errores
error$
  .pipe(
    catchError((error: unknown) => {
      console.error('Error capturado:', (error instanceof Error ? error.message : String(error)));
      return of('Valor de fallback después del error');
    })
  )
  .subscribe({
    next: (value) => console.log('Valor:', value),
    error: (err) => console.error('Error no manejado:', err),
    complete: () => console.log('Completado'),
  });

// Salida:
// Error capturado: Ocurrió un error
// Valor: Valor de fallback después del error
// Completado
```

## Diversas Estrategias de Manejo de Errores

### 1. Capturar Errores y Proporcionar Valores Alternativos

Use el operador `catchError` para capturar errores y proporcionar valores alternativos o flujos alternativos.

```ts
import { of, throwError } from 'rxjs';
import { catchError } from 'rxjs';

const source$ = throwError(() => new Error('Error en la obtención de datos'));

source$.pipe(
  catchError((error: unknown) => {
    console.error('Ocurrió un error:', (error instanceof Error ? error.message : String(error)));
    // Retornar datos alternativos
    return of({ isError: true, data: [], message: 'Mostrando datos predeterminados' });
  })
).subscribe(data => console.log('Resultado:', data));

// Salida:
// Ocurrió un error: Error en la obtención de datos
// Resultado: {isError: true, data: Array(0), message: 'Mostrando datos predeterminados'}
```

### 2. Reintentar si Ocurre un Error

Use el operador `retry` o `retryWhen` para reintentar el flujo si ocurre un error.

```ts
import { interval, throwError, of } from 'rxjs';
import { mergeMap, retry, tap } from 'rxjs';

let attemptCount = 0;

interval(1000).pipe(
  mergeMap(val => {
    if (++attemptCount <= 2) {
      return throwError(() => new Error(`Error #${attemptCount}`));
    }
    return of('¡Éxito!');
  }),
  tap(() => console.log('Ejecución:', attemptCount)),
  retry(2), // Reintentar hasta 2 veces
).subscribe({
  next: value => console.log('Valor:', value),
  error: err => console.error('Error final:', err.message),
});

// Salida:
// Ejecución: 3
// Valor: ¡Éxito!
// Ejecución: 4
// Valor: ¡Éxito!
// Ejecución: 5
// ...
```

### 3. Reintentar con Backoff Exponencial

Para solicitudes de red, por ejemplo, el "backoff exponencial", que aumenta gradualmente el intervalo de reintento, es efectivo.

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
    catchError((error: unknown) => {
      console.error('Todos los reintentos fallaron:', (error instanceof Error ? error.message : String(error)));
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
// Ocurrió un error: Error de red
// Intento de reintento 4 después de 10000ms
// Ocurrió un error: Error de red
// Intento de reintento 5 después de 10000ms
// Todos los reintentos fallaron: Se excedió el máximo de intentos de reintento
// Resultado: {error: true, message: 'Conexión fallida. Por favor, inténtelo de nuevo más tarde.'}
```

### 4. Liberación de Recursos Cuando Ocurre un Error

El operador `finalize` se utiliza para liberar recursos cuando un flujo **se completa o termina con un error**.
`finalize` es útil cuando desea asegurarse de que el procesamiento de limpieza se realice no solo cuando ocurre un error, sino también cuando se completa normalmente.

```ts
import { throwError } from 'rxjs';
import { catchError, finalize } from 'rxjs';

let isLoading = true;

throwError(() => new Error('Error de procesamiento'))
  .pipe(
    catchError((error: unknown) => {
      console.error('Manejo de errores:', (error instanceof Error ? error.message : String(error)));
      return throwError(() => error); // Relanzar error
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
// Manejo de errores: Error de procesamiento
// Error final: Error de procesamiento
// Restablecer estado de carga: false
```

## Patrones de Manejo de Errores

### Manejo de Errores Incluyendo Control de Visualización de Elementos de UI

```ts
import { of, throwError } from 'rxjs';
import { catchError, finalize, tap } from 'rxjs';

function fetchData(shouldFail = false) {
  // Mostrar indicador de carga
  showLoadingIndicator();

  // Obtención de datos (éxito o error)
  return (
    shouldFail
      ? throwError(() => new Error('Error de API'))
      : of({ name: 'Datos', value: 42 })
  ).pipe(
    tap((data) => {
      // Procesamiento en caso de éxito
      updateUI(data);
    }),
    catchError((error: unknown) => {
      // Actualizar UI en caso de error
      showErrorMessage((error instanceof Error ? error.message : String(error)));
      // Retornar datos vacíos o valor predeterminado
      return of({ name: 'Predeterminado', value: 0 });
    }),
    finalize(() => {
      // Ocultar indicador de carga independientemente del éxito o error
      hideLoadingIndicator();
    })
  );
}

// Funciones auxiliares para operaciones de UI
function showLoadingIndicator() {
  console.log('Mostrar carga');
}
function hideLoadingIndicator() {
  console.log('Ocultar carga');
}
function updateUI(data: { name: string; value: number }) {
  console.log('Actualizar UI:', data);
}
function showErrorMessage(message: any) {
  console.log('Mostrar error:', message);
}

// Ejemplo de uso
fetchData(true).subscribe();

// Salida:
// Mostrar carga
// Mostrar error: Error de API
// Ocultar carga
```

### Manejo de Múltiples Fuentes de Errores

```ts
import { forkJoin, of, throwError } from 'rxjs';
import { catchError, map } from 'rxjs';

// Simular múltiples solicitudes de API
function getUser() {
  return of({ id: 1, name: 'Taro Yamada' });
}

function getPosts() {
  return throwError(() => new Error('Error en la obtención de publicaciones'));
}

function getComments() {
  return throwError(() => new Error('Error en la obtención de comentarios'));
}

// Obtener todos los datos y permitir errores parciales
forkJoin({
  user: getUser().pipe(
    catchError((error: unknown) => {
      console.error('Error en la obtención de usuario:', (error instanceof Error ? error.message : String(error)));
      return of(null); // Retornar null en caso de error
    })
  ),
  posts: getPosts().pipe(
    catchError((error: unknown) => {
      console.error('Error en la obtención de publicaciones:', (error instanceof Error ? error.message : String(error)));
      return of([]); // Retornar array vacío en caso de error
    })
  ),
  comments: getComments().pipe(
    catchError((error: unknown) => {
      console.error('Error en la obtención de comentarios:', (error instanceof Error ? error.message : String(error)));
      return of([]); // Retornar array vacío en caso de error
    })
  ),
})
  .pipe(
    map((result) => ({
      ...result,
      // Agregar bandera que indica errores parciales
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
        'Falló la obtención de algunos datos, pero mostrando datos disponibles'
      );
    }
  });

// Salida:
// Error en la obtención de publicaciones: Error en la obtención de publicaciones
// Error en la obtención de comentarios: Error en la obtención de comentarios
// Resultado final: {user: {…}, posts: Array(0), comments: Array(0), hasErrors: true}
// Falló la obtención de algunos datos, pero mostrando datos disponibles
```

## Mejores Prácticas de Manejo de Errores

1. **Siempre capturar errores**: Siempre agregue manejo de errores en la cadena de Observable. Esto es especialmente importante para flujos de larga duración.

2. **Proporcionar mensajes de error significativos**: Los objetos de error deben incluir información que ayude a determinar la ubicación y causa del error.

3. **Liberar recursos adecuadamente**: Use `finalize` para asegurarse de que los recursos se liberen independientemente del éxito o fracaso.

4. **Considerar una estrategia de reintento**: Implementar una estrategia de reintento adecuada, especialmente para operaciones de red, mejorará la confiabilidad.

5. **Manejo de errores amigable para el usuario**: La UI debe proporcionar información que los usuarios puedan entender, en lugar de solo mostrar mensajes de error técnicos tal cual.

```ts
// Ejemplo: Conversión a mensajes de error amigables para el usuario
function getErrorMessage(error: any): string {
  if (error.status === 401) {
    return 'La sesión ha expirado. Por favor, inicie sesión de nuevo.';
  } else if (error.status === 404) {
    return 'No se encontró el recurso solicitado.';
  } else if (error.status >= 500) {
    return 'Ocurrió un error en el servidor. Por favor, inténtelo de nuevo más tarde.';
  }
  return 'Ocurrió un error inesperado.';
}
```

## Resumen

El manejo de errores en RxJS es una parte importante para asegurar la robustez de la aplicación. Con la combinación adecuada de operadores como `catchError`, `retry` y `finalize`, se pueden manejar una variedad de escenarios de error. Diseñe una estrategia integral de manejo de errores que vaya más allá de simplemente capturar errores para mejorar la experiencia del usuario.

## 🔗 Secciones Relacionadas

- **[Errores Comunes y Soluciones](/es/guide/anti-patterns/common-mistakes#9-error-suppression)** - Verifique los anti-patrones relacionados con el manejo de errores
- **[retry y catchError](/es/guide/error-handling/retry-catch)** - Métodos de uso detallados explicados
