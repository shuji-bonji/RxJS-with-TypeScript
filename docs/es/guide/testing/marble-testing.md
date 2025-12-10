---
description: Marble testing es una técnica que permite probar streams asíncronos de RxJS con una representación visual usando strings. Esta guía explica cuidadosamente la diferencia entre Cold y Hot Observable, las reglas de notación de marble, y cómo usar TestScheduler desde lo básico hasta ejemplos prácticos.
---

# Introducción al Marble Testing

RxJS proporciona una técnica llamada "marble testing" que permite probar el comportamiento de streams asíncronos **con una representación visual**.

En esta sección, aprenderemos los fundamentos del marble testing a través de ejemplos simples.

## ¿Qué es la Notación de Marble?

La notación de marble es una forma de representar **el paso del tiempo y la ocurrencia de eventos** con strings.

### Reglas básicas

Estos símbolos se usan para representar el paso del tiempo y la ocurrencia de eventos.

| Símbolo | Significado |
|:----|:----|
| `-` | El tiempo pasa (avanzar 1 frame) |
| `a`, `b`, `c` | Valores emitidos (caracteres arbitrarios) |
| `|` | Completar |
| `#` | Error |

Por ejemplo.

```text
--a-b--c-|
```
Esto significa:
- Esperar 2 frames y `a` es emitido
- `b` después de 1 frame
- `c` 2 frames después
- Completar después de 1 frame más

## Diferencia entre Cold y Hot

### Cold Observable

Cold Observable es "reproducido desde el principio con cada suscripción".

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

      expectObservable(source$).toBe('--a--b--c|');
    });
  });
});

```

### Hot Observable

Un Hot Observable es un stream que "ya está en progreso".
Si se suscribe a mitad del stream, solo recibirá valores desde ese punto en adelante.

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
    testScheduler.run(({ hot, expectObservable }) => {
      const source$ = hot('--a--b--c|');

      expectObservable(source$, '----^').toBe('-----b--c|');
    });
  });
});

```

## Un ejemplo simple de marble test

Por ejemplo, para probar el operador `debounceTime`

```ts
import { debounceTime } from 'rxjs';
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
      const source$ = cold('--a--b----c|');
      const result$ = source$.pipe(
        debounceTime(20)
      );

      const expected =    '-----------(c|)';

      expectObservable(result$).toBe(expected, { c: 'c' });
    });
  });
});

```

Aquí verificamos que solo la última `c` emitida es la salida.

## Notas

- Un carácter en la notación de marble representa **un frame (10ms)** por defecto (configurable dependiendo del entorno)
- Los operadores dependientes del tiempo como `debounceTime`, `delay`, `interval`, etc. **funcionan bien con marble tests**
- Use `expectObservable` para validar la salida del stream
- `expectSubscriptions` es una característica avanzada que valida el tiempo de las suscripciones, pero no se cubre aquí

## Resumen

Marble testing es una técnica muy poderosa que hace que las pruebas de código RxJS sean **visibles e intuitivas**.

- **Ser consciente de la diferencia entre Cold y Hot**
- **Representar el paso del tiempo y eventos como strings**
- **Los streams asíncronos complejos también se pueden probar claramente**

¡Comience con un marble test simple para practicar!
