---
description: Esta página introduce seis escenarios comunes de depuración para RxJS, incluyendo valores que no fluyen, valores diferentes a los esperados, suscripción que no se completa, fugas de memoria, errores no detectados y seguimiento de reintentos, junto con problemas prácticos y soluciones.
---

# Escenarios comunes de depuración

Se describen problemas típicos encontrados en el desarrollo de RxJS y sus soluciones con ejemplos de código concretos.

## Escenario 1: Los valores no fluyen

- **Síntoma**: Hago `subscribe` y no se emite un solo valor.


### Causa 1: Olvidó suscribirse al Cold Observable.

El Cold Observable no se ejecutará hasta que se suscriba a él.

```ts
import { interval } from 'rxjs';
import { map } from 'rxjs';

// ❌ No se ejecuta nada porque no está suscrito
const numbers$ = interval(1000).pipe(
  map(x => {
    console.log('Esta línea no se ejecuta');
    return x * 2;
  })
);

// ✅ Ejecutado al suscribirse
numbers$.subscribe(value => console.log('Valor:', value));
```

### Causa 2: Subject completado

Una vez que se completa un Subject, no recibirá valores en suscripciones posteriores.

```ts
import { Subject } from 'rxjs';

const subject = new Subject<number>();

subject.complete(); // Completar

// ❌ La suscripción después de completar no recibe valor
subject.subscribe(value => console.log('Esta línea no se ejecuta'));

// ✅ Suscribirse antes de completar
const subject2 = new Subject<number>();
subject2.subscribe(value => console.log('Valor:', value));
subject2.next(1); // Valor: 1
subject2.complete();
```

### Causa 3: Filtrado con condiciones incorrectas

Las condiciones de filtrado pueden ser demasiado estrictas y excluir todos los valores.

```ts
import { of } from 'rxjs';
import { filter, tap } from 'rxjs';

of(1, 2, 3, 4, 5)
  .pipe(
    tap(value => console.log('Antes de filter:', value)),
    filter(x => x > 10), // Todos excluidos
    tap(value => console.log('Después de filter:', value)) // Esta línea no se ejecuta
  )
  .subscribe({
    next: value => console.log('Valor final:', value),
    complete: () => console.log('Completado (sin valor)')
  });

// Salida:
// Antes de filter: 1
// Antes de filter: 2
// Antes de filter: 3
// Antes de filter: 4
// Antes de filter: 5
// Completado (sin valor)
```

### Técnicas de depuración

Use el operador `tap` para ver qué valores fluyen en cada paso.

```ts
import { of, EMPTY } from 'rxjs';
import { filter, tap, defaultIfEmpty } from 'rxjs';

of(1, 2, 3, 4, 5)
  .pipe(
    tap(value => console.log('🔵 Entrada:', value)),
    filter(x => x > 10),
    tap(value => console.log('🟢 Pasó el filtro:', value)),
    defaultIfEmpty('Sin valor') // Valor predeterminado si no hay valor
  )
  .subscribe(value => console.log('✅ Salida:', value));

// Salida:
// 🔵 Entrada: 1
// 🔵 Entrada: 2
// 🔵 Entrada: 3
// 🔵 Entrada: 4
// 🔵 Entrada: 5
// ✅ Salida: Sin valor
```

## Escenario 2: Se emite un valor diferente al esperado

- **Síntoma**: Se emite un valor diferente al esperado.

### Causa 1: El operador está en el orden incorrecto.

El resultado depende del orden en que se aplican los operadores.

```ts
import { of } from 'rxjs';
import { map, filter } from 'rxjs';

// ❌ Resultado diferente al esperado
of(1, 2, 3, 4, 5)
  .pipe(
    map(x => x * 2),     // 2, 4, 6, 8, 10
    filter(x => x < 5)   // Solo 2, 4 pasan
  )
  .subscribe(value => console.log('Resultado:', value));
// Salida: 2, 4

// ✅ Orden correcto
of(1, 2, 3, 4, 5)
  .pipe(
    filter(x => x < 5),  // Solo 1, 2, 3, 4 pasan
    map(x => x * 2)      // 2, 4, 6, 8
  )
  .subscribe(value => console.log('Resultado:', value));
// Salida: 2, 4, 6, 8
```

### Causa 2: Cambios no deseados debido a referencias compartidas

Debido a que los objetos de JavaScript se pasan por referencia, es posible modificar el objeto original.

```ts
import { of } from 'rxjs';
import { map } from 'rxjs';

interface User {
  id: number;
  name: string;
}

const user: User = { id: 1, name: 'Alice' };

of(user)
  .pipe(
    // ❌ Modifica el objeto original directamente
    map(u => {
      u.name = 'Bob'; // El objeto original se modifica
      return u;
    })
  )
  .subscribe(value => console.log('Después del cambio:', value));

console.log('Objeto original:', user); // { id: 1, name: 'Bob' }

// ✅ Crear un nuevo objeto
of(user)
  .pipe(
    map(u => ({ ...u, name: 'Charlie' })) // Nuevo objeto con sintaxis spread
  )
  .subscribe(value => console.log('Después del cambio:', value));

console.log('Objeto original:', user); // { id: 1, name: 'Alice' } (no modificado)
```

### Causa 3: Sincronización del procesamiento asíncrono

El orden de finalización del procesamiento asíncrono puede ser diferente al esperado.

```ts
import { of, delay } from 'rxjs';
import { mergeMap, tap } from 'rxjs';

// ❌ No espera a que se complete el procesamiento asíncrono
of(1, 2, 3)
  .pipe(
    tap(value => console.log('Inicio:', value)),
    mergeMap(value =>
      of(value * 2).pipe(
        delay(100 - value * 10) // Los valores más grandes se completan más rápido
      )
    )
  )
  .subscribe(value => console.log('Completado:', value));

// Salida:
// Inicio: 1
// Inicio: 2
// Inicio: 3
// Completado: 3  ← Retraso más corto
// Completado: 2
// Completado: 1  ← Retraso más largo

// ✅ Garantizar el orden
import { concatMap } from 'rxjs';

of(1, 2, 3)
  .pipe(
    tap(value => console.log('Inicio:', value)),
    concatMap(value =>  // mergeMap → concatMap
      of(value * 2).pipe(delay(100 - value * 10))
    )
  )
  .subscribe(value => console.log('Completado:', value));

// Salida:
// Inicio: 1
// Completado: 1
// Inicio: 2
// Completado: 2
// Inicio: 3
// Completado: 3
```

## Escenario 3: La suscripción no se completa (stream infinito)

- **Síntoma**: `complete` no se llama y el stream no se termina

Debe completarlo explícitamente, ya que `interval`, `fromEvent`, etc. siguen emitiendo valores indefinidamente.

```ts
import { interval } from 'rxjs';
import { tap } from 'rxjs';

// ❌ interval continúa emitiendo valores indefinidamente
interval(1000)
  .pipe(
    tap(value => console.log('Valor:', value))
  )
  .subscribe({
    complete: () => console.log('Esta línea no se ejecuta')
  });

// ✅ Completar explícitamente con take
import { take } from 'rxjs';

interval(1000)
  .pipe(
    take(5), // Completar después de 5 valores
    tap(value => console.log('Valor:', value))
  )
  .subscribe({
    complete: () => console.log('Completado')
  });
```

### Técnicas de depuración

Establezca un tiempo de espera para detener el stream infinito al depurar.

```ts
import { interval, timer } from 'rxjs';
import { tap, takeUntil } from 'rxjs';

// Establecer tiempo de espera para depuración
const stop$ = timer(5000); // Completar después de 5 segundos

interval(1000)
  .pipe(
    takeUntil(stop$),
    tap({
      next: value => console.log('Valor:', value),
      complete: () => console.log('Detenido por tiempo de espera')
    })
  )
  .subscribe();
```

## Escenario 4: Fuga de memoria (olvidó cancelar la suscripción)

- **Síntoma**: La aplicación se vuelve gradualmente más lenta

### Causa: Suscripciones no canceladas que ya no son necesarias

Se produce una fuga de memoria cuando una suscripción permanece después de que se destruye un componente o servicio.

```ts
import { interval } from 'rxjs';

class UserComponent {
  private subscription: any;

  ngOnInit() {
    // ❌ Olvidó cancelar la suscripción
    interval(1000).subscribe(value => {
      console.log('Valor:', value); // Continúa ejecutándose después de que se destruye el componente
    });
  }

  ngOnDestroy() {
    // Sin cancelación de suscripción
  }
}

// ✅ Administrar las suscripciones adecuadamente
class UserComponentFixed {
  private subscription: any;

  ngOnInit() {
    this.subscription = interval(1000).subscribe(value => {
      console.log('Valor:', value);
    });
  }

  ngOnDestroy() {
    // Cancelar la suscripción cuando se destruye el componente
    if (this.subscription) {
      this.subscription.unsubscribe();
    }
  }
}
```

**Patrón recomendado: use `takeUntil`**.

El patrón `takeUntil` se puede usar para automatizar las cancelaciones de suscripción.

```ts
import { interval, Subject } from 'rxjs';
import { takeUntil } from 'rxjs';

class UserComponentBest {
  private destroy$ = new Subject<void>();

  ngOnInit() {
    // ✅ Cancelar automáticamente la suscripción con takeUntil
    interval(1000)
      .pipe(
        takeUntil(this.destroy$)
      )
      .subscribe(value => console.log('Valor:', value));
  }

  ngOnDestroy() {
    this.destroy$.next();
    this.destroy$.complete();
  }
}
```

### Detección de fugas de memoria

Rastree el número de suscripciones con un operador personalizado.

```ts
import { interval } from 'rxjs';
import { tap } from 'rxjs';

let subscriptionCount = 0;

const trackSubscriptions = <T>() =>
  tap<T>({
    subscribe: () => {
      subscriptionCount++;
      console.log('📈 Suscripciones:', subscriptionCount);
    },
    unsubscribe: () => {
      subscriptionCount--;
      console.log('📉 Suscripciones:', subscriptionCount);
    }
  });

// Ejemplo de uso
const stream$ = interval(1000).pipe(
  trackSubscriptions()
);

const sub1 = stream$.subscribe();
// Salida: 📈 Suscripciones: 1

const sub2 = stream$.subscribe();
// Salida: 📈 Suscripciones: 2

setTimeout(() => {
  sub1.unsubscribe();
  // Salida: 📉 Suscripciones: 1
}, 3000);
```

## Escenario 5: No detecta un error

- **Síntoma**: Ocurre un error, pero no se muestra y se ignora

Sin un manejador de errores, el error puede quedar sin detectar.

```ts
import { of, throwError } from 'rxjs';
import { mergeMap, catchError } from 'rxjs';

// ❌ El error se suprime porque no hay manejo de errores
of(1, 2, 3)
  .pipe(
    mergeMap(value => {
      if (value === 2) {
        return throwError(() => new Error('Error'));
      }
      return of(value);
    })
  )
  .subscribe(); // Sin manejador de errores

// ✅ Manejo de errores adecuado
of(1, 2, 3)
  .pipe(
    mergeMap(value => {
      if (value === 2) {
        return throwError(() => new Error('Error'));
      }
      return of(value);
    }),
    catchError(error => {
      console.error('🔴 Error capturado:', error.message);
      return of(-1); // Valor de respaldo
    })
  )
  .subscribe({
    next: value => console.log('Valor:', value),
    error: error => console.error('🔴 Error en subscribe:', error)
  });

// Salida:
// Valor: 1
// 🔴 Error capturado: Error
// Valor: -1
```

### Configurar manejador de errores global

Se puede configurar un manejador global para capturar todos los errores pendientes.

```ts
import { Observable } from 'rxjs';

// Capturar todos los errores no manejados
const originalCreate = Observable.create;

Observable.create = function(subscribe: any) {
  return originalCreate.call(this, (observer: any) => {
    try {
      return subscribe(observer);
    } catch (error) {
      console.error('🔴 Error no manejado:', error);
      observer.error(error);
    }
  });
};
```

## Escenario 6: Quiero rastrear los intentos de reintento

- **Síntoma**: Estoy usando el operador `retry`, pero no sé cuántos reintentos estoy obteniendo.

Al reintentar automáticamente cuando ocurre un error, rastrear cuántos reintentos se realizan realmente facilitaría la depuración y el registro.

### Depuración básica de reintentos

Use `retryWhen` para registrar el número de reintentos.

```ts
import { throwError, of, timer } from 'rxjs';
import { retryWhen, mergeMap, tap } from 'rxjs';

throwError(() => new Error('Error temporal'))
  .pipe(
    retryWhen((errors) =>
      errors.pipe(
        mergeMap((error, index) => {
          const retryCount = index + 1;
          console.log(`🔄 Intento de reintento ${retryCount}`);

          if (retryCount > 2) {
            console.log('❌ Número máximo de reintentos alcanzado');
            throw error;
          }

          return timer(1000);
        })
      )
    )
  )
  .subscribe({
    next: value => console.log('✅ Éxito:', value),
    error: error => console.log('🔴 Error final:', error.message)
  });

// Salida:
// 🔄 Intento de reintento 1
// 🔄 Intento de reintento 2
// 🔄 Intento de reintento 3
// ❌ Número máximo de reintentos alcanzado
// 🔴 Error final: Error temporal
```

> [!TIP]
> Para patrones de implementación más detallados sobre la depuración de reintentos, consulte la sección "Depuración de reintentos" de [retry y catchError](/es/guide/error-handling/retry-catch#debugging-retries).
> - Seguimiento básico usando el callback de error tap
> - Registro detallado con retryWhen
> - Exponential backoff y registro
> - Objeto de configuración retry de RxJS 7.4+

## Resumen

Soluciones a escenarios comunes de depuración

- ✅ **Los valores no fluyen** → olvidó suscribirse, verifique las condiciones de filtrado
- ✅ **Valor diferente al esperado** → tenga cuidado con el orden de operadores, compartir referencias
- ✅ **Suscripción no completada** → use `take` o `takeUntil` para streams infinitos
- ✅ **Fuga de memoria** → cancelación automática de suscripción con patrón `takeUntil`
- ✅ **Errores no detectados** → implemente un manejo de errores adecuado
- ✅ **Seguimiento de reintentos** → registro con `retryWhen` u objeto de configuración

## Páginas relacionadas

- [Estrategias básicas de depuración](/es/guide/debugging/) - Cómo usar el operador tap y las herramientas de desarrollo
- [Herramientas de depuración personalizadas](/es/guide/debugging/custom-tools) - Streams con nombre, operadores de depuración
- [Depuración de rendimiento](/es/guide/debugging/performance) - Monitoreo de suscripciones, verificación de uso de memoria
- [Manejo de errores](/es/guide/error-handling/strategies) - Estrategias de manejo de errores
