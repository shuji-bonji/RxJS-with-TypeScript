---
description: "Aprenda a manejar la finalización de flujos y la limpieza de recursos en RxJS usando finalize y complete: Técnicas esenciales para prevenir fugas de memoria"
titleTemplate: ':title'
---
# finalize y complete - Liberación de Recursos y Procesamiento de Finalización de Flujo

En RxJS, es importante gestionar adecuadamente la finalización de flujos y la liberación de recursos. Esta página explica cómo funcionan el operador `finalize` y las notificaciones `complete`.

## finalize - Operador para Liberación de Recursos

El operador `finalize` es el operador que ejecuta el código de limpieza especificado cuando el Observable sale con **complete, error o unsubscribe**.
finalize siempre se llama **solo una vez** al final del flujo y nunca más de una vez.

[🌐 Documentación Oficial de RxJS - finalize](https://rxjs.dev/api/index/function/finalize)

### Uso Básico de finalize

```ts
import { of } from 'rxjs';
import { finalize, tap } from 'rxjs';

// Variable para gestionar el estado de carga
let isLoading = true;

// Flujo que tiene éxito
of('data')
  .pipe(
    tap((data) => console.log('Procesando datos:', data)),
    // Ejecutado en todos los casos: éxito, fracaso o cancelación
    finalize(() => {
      isLoading = false;
      console.log('Estado de carga restablecido:', isLoading);
    })
  )
  .subscribe({
    next: (value) => console.log('Valor:', value),
    complete: () => console.log('Completado'),
  });

// Salida:
// Procesando datos: data
// Valor: data
// Completado
// Estado de carga restablecido: false
```

### finalize en Error

```ts
import { throwError } from 'rxjs';
import { finalize, catchError } from 'rxjs';

let isLoading = true;

throwError(() => new Error('Error en la obtención de datos'))
  .pipe(
    catchError((err) => {
      console.error('Manejo de errores:', err.message);
      throw err; // Relanzar error
    }),
    finalize(() => {
      isLoading = false;
      console.log('Liberación de recursos después del error:', isLoading);
    })
  )
  .subscribe({
    next: (value) => console.log('Valor:', value),
    error: (err) => console.error('Error en suscriptor:', err.message),
    complete: () => console.log('Completado'), // No llamado en error
  });

// Salida:
// Manejo de errores: Error en la obtención de datos
// Error en suscriptor: Error en la obtención de datos
// Liberación de recursos después del error: false
```

### finalize en Unsubscribe

```ts
import { interval } from 'rxjs';
import { finalize } from 'rxjs';

let resource = 'Activo';

// Contar cada segundo
const subscription = interval(1000)
  .pipe(
    finalize(() => {
      resource = 'Liberado';
      console.log('Estado del recurso:', resource);
    })
  )
  .subscribe((count) => {
    console.log('Conteo:', count);

    // Cancelar manualmente la suscripción después de contar 3 veces
    if (count >= 2) {
      subscription.unsubscribe();
    }
  });

// Salida:
// Conteo: 0
// Conteo: 1
// Conteo: 2
// Estado del recurso: Liberado
```

Finalize es útil no solo cuando ocurre un error, sino también cuando desea asegurar el procesamiento de limpieza tras la finalización exitosa o la cancelación manual de suscripción.

## complete - Notificación de Finalización Exitosa del Flujo

Cuando un Observable termina exitosamente, se invoca el callback `complete` del Observer. Este es el último paso en el ciclo de vida del Observable.

### complete Automático

Algunos Observables se completan automáticamente cuando se cumplen ciertas condiciones.

```ts
import { of } from 'rxjs';
import { take } from 'rxjs';

// Las secuencias finitas se completan automáticamente
of(1, 2, 3).subscribe({
  next: (value) => console.log('Valor:', value),
  complete: () => console.log('Flujo finito completado'),
});

// Flujo limitado con interval + take
interval(1000)
  .pipe(
    take(3) // Completar después de obtener 3 valores
  )
  .subscribe({
    next: (value) => console.log('Conteo:', value),
    complete: () => console.log('Flujo limitado completado'),
  });

// Salida:
// Valor: 1
// Valor: 2
// Valor: 3
// Flujo finito completado
// Conteo: 0
// Conteo: 1
// Conteo: 2
// Flujo limitado completado

```

### complete Manual

Para Subject y personalizados, complete se puede llamar manualmente.

```ts
import { Subject } from 'rxjs';

const subject = new Subject<number>();

subject.subscribe({
  next: (value) => console.log('Valor:', value),
  complete: () => console.log('Subject completado'),
});

subject.next(1);
subject.next(2);
subject.complete(); // Completar manualmente
subject.next(3); // Ignorado después de la finalización

// Salida:
// Valor: 1
// Valor: 2
// Subject completado
```

## Diferencia entre finalize y complete

Es importante entender las diferencias importantes.

1. **Temporización de Ejecución**
   - `complete`: llamado solo cuando un Observable se completa **exitosamente**
   - `finalize`: llamado cuando el Observable termina con **finalización, error o cancelación de suscripción**

2. **Uso**
   - `complete`: Recibir notificación de una finalización exitosa (procesamiento en caso de éxito)
   - `finalize`: Asegurar que los recursos se liberen o se limpien (procesamiento que debe realizarse independientemente del éxito o fracaso)

## Casos de Uso Prácticos

### Llamadas de API y Gestión de Estado de Carga

```ts
import { ajax } from 'rxjs/ajax';
import { finalize, catchError } from 'rxjs';
import { of } from 'rxjs';

// Estado de carga
let isLoading = false;

function fetchData(id: string) {
  // Iniciar carga
  isLoading = true;
  const loading = document.createElement('p');
  loading.style.display = 'block';
  document.body.appendChild(loading);
  // document.getElementById('loading')!.style.display = 'block';

  // Solicitud de API
  return ajax.getJSON(`https://jsonplaceholder.typicode.com/posts/${id}`).pipe(
    catchError((error) => {
      console.error('Error de API:', error);
      return of({ error: true, message: 'Falló la obtención de datos' });
    }),
    // Finalizar carga independientemente del éxito o fracaso
    finalize(() => {
      isLoading = false;
      loading!.style.display = 'none';
      console.log('Restablecimiento del estado de carga completado');
    })
  );
}

// Ejemplo de uso
fetchData('123').subscribe({
  next: (data) => console.log('Datos:', data),
  complete: () => console.log('Obtención de datos completada'),
});

// Salida:
//  Error de API: AjaxErrorImpl {message: 'ajax error', name: 'AjaxError', xhr: XMLHttpRequest, request: {…}, status: 0, …}
//  Datos: {error: true, message: 'Falló la obtención de datos'}
//  Obtención de datos completada
//  Restablecimiento del estado de carga completado
//   GET https://jsonplaceholder.typicode.com/posts/123 net::ERR_NAME_NOT_RESOLVED
```

### Limpieza de Recursos

```ts
import { interval } from 'rxjs';
import { finalize, takeUntil } from 'rxjs';
import { Subject } from 'rxjs';

class ResourceManager {
  private destroy$ = new Subject<void>();
  private timerId: number | null = null;

  constructor() {
    // Inicializar algún recurso
    this.timerId = window.setTimeout(() => console.log('Ejecución de temporizador'), 10000);

    // Procesamiento periódico
    interval(1000)
      .pipe(
        // Detener en la destrucción del componente
        takeUntil(this.destroy$),
        // Asegurar la liberación de recursos
        finalize(() => {
          console.log('Intervalo detenido');
        })
      )
      .subscribe((count) => {
        console.log('Ejecutando...', count);
      });
  }

  dispose() {
    // Procesamiento de disposición
    if (this.timerId) {
      window.clearTimeout(this.timerId);
      this.timerId = null;
    }

    // Señal de detención de flujo
    this.destroy$.next();
    this.destroy$.complete();

    console.log('Disposición del gestor de recursos completada');
  }
}

// Ejemplo de uso
const manager = new ResourceManager();

// Disponer después de 5 segundos
setTimeout(() => {
  manager.dispose();
}, 5000);

// Salida:
// Ejecutando... 0
// Ejecutando... 1
// Ejecutando... 2
// Ejecutando... 3
// Ejecutando... 4
// Intervalo detenido
// Disposición del gestor de recursos completada
```

[📘 RxJS Oficial: takeUntil()](https://rxjs.dev/api/index/function/takeUntil)

## Mejores Prácticas

1. **Siempre liberar recursos**: use `finalize` para asegurar la limpieza cuando el flujo termine
2. **Gestión del estado de carga**: siempre restablezca el estado de carga usando `finalize`
3. **Gestión del ciclo de vida de componentes**: use `takeUntil` en combinación con `finalize` para limpiar recursos cuando los componentes se destruyen (este patrón se recomienda especialmente para Angular)
4. **Usar con manejo de errores**: Combine `catchError` y `finalize` para proporcionar manejo de fallback y limpieza confiable después de errores
5. **Conocer el estado de finalización**: use el callback `complete` para determinar si el flujo se ha completado exitosamente

## Resumen

`finalize` y `complete` son herramientas importantes para la gestión de recursos y la finalización de procesamiento en RxJS. `finalize` es ideal para la liberación de recursos porque asegura que el flujo se ejecute sin importar cómo termine. Por otro lado, `complete` se usa cuando desea realizar procesamiento de salida normal. Al combinar estas herramientas apropiadamente, puede prevenir fugas de memoria y construir aplicaciones confiables.
