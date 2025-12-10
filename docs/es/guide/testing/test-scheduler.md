---
description: TestScheduler es una herramienta poderosa que permite probar operadores basados en tiempo de RxJS usando tiempo virtual. Esta guía explica la notación de marble, cómo manejar Cold y Hot Observable, y cómo realizar pruebas unitarias precisas de procesos dependientes del tiempo como debounceTime y delay.
---

# Pruebas con TestScheduler

El `TestScheduler` de RxJS es una herramienta poderosa para probar con precisión operadores basados en tiempo. Este capítulo explica sistemáticamente cómo probar utilizando TestScheduler.

## ¿Qué es TestScheduler?

Normalmente, Observable funciona de manera dependiente del tiempo. Por ejemplo, `delay()` y `debounceTime()` son operadores que esperan un cierto tiempo.
Como es ineficiente esperar realmente en las pruebas, `TestScheduler` es un mecanismo para probar inmediatamente usando tiempo virtual.

> [!TIP]
> TestScheduler usa "tiempo virtual", por lo que no hay necesidad de esperar tiempo real.

## Configuración básica de TestScheduler

Esta es la configuración de prueba básica usando TestScheduler.

```ts
import { TestScheduler } from 'rxjs/testing';
import { describe, it, beforeEach, expect } from 'vitest';

describe('Fundamentos de TestScheduler', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('Prueba simple', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source$ = cold('--a--b--c|');
      const expected =  '--a--b--c|';

      expectObservable(source$).toBe(expected);
    });
  });
});
```

- `cold()`: Crear un Cold Observable donde el stream comienza independientemente para cada suscripción
- `hot()`: Crear un Hot Observable donde el stream ya está en progreso
- `expectObservable()`: verificar la salida del Observable con notación de marble


## Cold Observable y Hot Observable

|Tipo|Características|Uso|
|:---|:---|:---|
|Cold Observable|Flujo de datos desde el principio para cada suscripción|Solicitud HTTP, etc.|
|Hot Observable|El flujo de datos ya ha comenzado y se comparte con suscriptores|Eventos de usuario, WebSockets, etc.|


## Fundamentos de la notación de marble

La notación de marble es un método de representar el paso del tiempo en Observable como un string.

|Símbolo|Significado|
|:---|:---|
|`-`|Tiempo transcurrido (un frame)|
|`a`, `b`, `c`|valor emitido|
|`|`|Completado|
|`#`|Error|
|`() `|Múltiples valores emitidos simultáneamente (múltiples eventos)|

#### Ejemplo

```
--a--b--c|    // a después de 2 frames, luego b, c, completar
```


## Ejemplo de prueba con tiempo virtual

### Probar con debounceTime

Probar el operador debounceTime usando tiempo virtual.

```ts
import { describe, it, expect, beforeEach } from 'vitest';
import { debounceTime, map } from 'rxjs';
import { TestScheduler } from 'rxjs/testing';

describe('Probar con tiempo virtual', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('Probar operador debounceTime', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source$ = cold('--a--b----c|');
      const result$ = source$.pipe(
        debounceTime(20),
        map(x => x.toUpperCase())
      );
      const expected =    '-----------(C|)';  // ← ¡Esto!

      expectObservable(result$).toBe(expected, { B: 'B', C: 'C' });
    });
  });
});
```


## Probar manejo de errores

Probar el comportamiento cuando ocurre un error.

```ts
import { describe, it, expect, beforeEach } from 'vitest';
import { catchError} from 'rxjs';
import { TestScheduler } from 'rxjs/testing';
import { of } from 'rxjs';

describe('Prueba de manejo de errores', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('Capturar error con catchError', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source$ = cold('--a--#');
      const result$ = source$.pipe(
        catchError(() => of('X'))
      );

      const expected =    '--a--(X|)';

      expectObservable(result$).toBe(expected);
    });
  });
});
```


## Resumen

- TestScheduler permite probar sin esperar tiempo real
- Comprender la diferencia entre Cold/Hot Observable y usarlo de manera diferente
- Visualizar el paso del tiempo usando notación de marble
- Probar incluso streams asíncronos complejos con precisión

> [!NEXT]
> A continuación, aprenderemos marble testing más avanzado (personalización de strings de marble y combinaciones de múltiples streams).
