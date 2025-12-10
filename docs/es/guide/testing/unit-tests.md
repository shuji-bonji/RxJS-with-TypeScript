---
description: Las pruebas unitarias en RxJS utilizan técnicas síncronas, asíncronas y controladas por tiempo para construir una estrategia de pruebas robusta utilizando TestScheduler, marble testing, y mocks y stubs.
---

# Pruebas Unitarias para RxJS

El código que usa RxJS implica mucho procesamiento asíncrono y requiere un enfoque diferente al de los métodos de prueba tradicionales. Esta guía describe tanto técnicas básicas como avanzadas para probar efectivamente código que usa RxJS.

## Probar Observable Síncrono

Comencemos con el caso más simple: probar un Observable que se completa de forma síncrona.

```ts
import { Observable, of } from 'rxjs';
import { map } from 'rxjs';
import { describe, it, expect } from 'vitest';

// Función bajo prueba
function doubleValues(input$: Observable<number>) : Observable<number>{
  return input$.pipe(
    map(x => x * 2)
  );
}

describe('Pruebas básicas de Observable', () => {
  it('Duplica valores', () => {
    // Observable de prueba
    const source$ = of(1, 2, 3);
    const result$ = doubleValues(source$);

    // Resultado esperado
    const expected = [2, 4, 6];
    const actual: number[] = [];

    // Ejecución y verificación
    result$.subscribe({
      next: (value) => actual.push(value),
      complete: () => {
        expect(actual).toEqual(expected);
      }
    });
  });
});
```

## Cómo probar un Observable asíncrono

Para Observable asíncrono, aproveche el soporte asíncrono del framework de pruebas.

```ts
import { Observable, timer } from 'rxjs';
import { map, take } from 'rxjs';
import { describe, it, expect } from 'vitest';

// Función asíncrona bajo prueba
function getDelayedValues(): Observable<number> {
  return timer(0, 100).pipe(
    map(x => x + 1),
    take(3)
  );
}

describe('Prueba de Observable asíncrono', () => {
  it('Recibe valores asíncronos en orden', (done: Function) => {
    const result$ = getDelayedValues();
    const expected = [1, 2, 3];
    const actual: number[] = [];

    result$.subscribe({
      next: (value) => actual.push(value),
      complete: () => {
        expect(actual).toEqual(expected);
        done();
      }
    });
  });
});
```

## Pruebas asíncronas con transformación de Promise

Otro método es convertir un Observable a una Promise usando `firstValueFrom()` o `lastValueFrom()` y utilizar async/await del JS/TS moderno.

```ts
import { Observable, of } from 'rxjs';
import { map, delay, toArray } from 'rxjs';
import { describe, it, expect } from 'vitest';
import { lastValueFrom } from 'rxjs';

// Función bajo prueba
function processWithDelay(input$: Observable<number>) {
  return input$.pipe(
    map(x => x * 10),
    delay(100),
    toArray()
  );
}

describe('Pruebas con conversión de Promise', () => {
  it('Esperar procesamiento de delay antes de validación', async () => {
    const source$ = of(1, 2, 3);
    const result$ = processWithDelay(source$);

    // Convertir Observable a promise
    const result = await lastValueFrom(result$);

    // Resultado esperado
    expect(result).toEqual([10, 20, 30]);
  });
});
```

## Utilizar TestScheduler

RxJS proporciona un scheduler especial llamado `TestScheduler` que puede usarse para probar eficientemente operadores basados en tiempo.

```ts
import { TestScheduler } from 'rxjs/testing';
import { map, debounceTime } from 'rxjs';
import { describe, it, beforeEach } from 'vitest';

describe('Usar TestScheduler', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('Probar debounceTime', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source = cold('a--b--c--d|', { a: 1, b: 2, c: 3, d: 4 });
      const result = source.pipe(
        debounceTime(20),
        map(x => x * 10)
      );

      const expected = '----------(d|)';

      expectObservable(result).toBe(expected, { d: 40 });
    });
  });
});
```

> [!NOTE]
> Notación de Marble Test
> Al usar `TestScheduler`, use diagramas de marble para representar el paso del tiempo.

## Hacer el tiempo manipulable

Al probar código dependiente del tiempo (delay, debounceTime, etc.), use el `TestScheduler` para controlar el tiempo.

```ts
import { TestScheduler } from 'rxjs/testing';
import { interval } from 'rxjs';
import { take, map } from 'rxjs';
import { describe, it, beforeEach } from 'vitest';

describe('Control de tiempo', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('Avance rápido del tiempo para pruebas', () => {
    testScheduler.run(({ expectObservable }) => {
      const source = interval(1000).pipe(
        take(3),
        map(x => x + 1)
      );

      // Realmente toma 3 segundos, pero se ejecuta inmediatamente en entorno de prueba
      const expected = '1s a 999ms b 999ms (c|)';
      const values = { a: 1, b: 2, c: 3 };

      expectObservable(source).toBe(expected, values);
    });
  });
});
```

## Probar manejo de errores (versión TestScheduler)

También es importante probar el comportamiento del Observable cuando ocurre un error.

```ts
import { TestScheduler } from 'rxjs/testing';
import { throwError, of } from 'rxjs';
import { catchError } from 'rxjs';

describe('Prueba de manejo de errores', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('Cuando Observable notifica un error', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source = cold('  --a--b--#');
      const expected =     '--a--b--#';

      expectObservable(source).toBe(expected);
    });
  });

  it('Cuando catchError captura error y lo reemplaza con un valor', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source = cold('  --a--b--#');
      const handled = source.pipe(
        catchError(() => of('X'))
      );

      const expected =     '--a--b--(X|)';

      expectObservable(handled).toBe(expected);
    });
  });
});
```

## Marble test

Para probar streams complejos, use un diagrama de marble para representar intuitivamente las expectativas de prueba.

### Hot Observable vs. Cold Observable

TestScheduler permite la creación de dos tipos de Observables: hot y cold. Es importante entender esta diferencia al probar.

```ts
import { TestScheduler } from 'rxjs/testing';
import { Subject } from 'rxjs';
import { describe, it, beforeEach, expect } from 'vitest';

describe('Prueba de Hot vs Cold Observable', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('Cold Observable crea streams independientes para cada suscripción', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      // Cold Observable (independiente para cada suscriptor)
      const source = cold('--a--b--c|', { a: 1, b: 2, c: 3 });

      // Primera suscripción
      expectObservable(source).toBe('--a--b--c|', { a: 1, b: 2, c: 3 });

      // Segunda suscripción (comienza desde el principio)
      expectObservable(source).toBe('--a--b--c|', { a: 1, b: 2, c: 3 });
    });
  });

  it('Hot Observable comparte streams entre suscriptores', () => {
    testScheduler.run(({ hot, expectObservable }) => {
      // Hot Observable (compartido entre suscriptores)
      const source = hot('--a--b--c|', { a: 1, b: 2, c: 3 });

      // Suscribirse tarde (recibe solo valores después de que comienza la suscripción)
      expectObservable(source, '----^').toBe('-----b--c|', { b: 2, c: 3 });

      // Suscribirse desde el principio (recibe todos los valores)
      expectObservable(source).toBe('--a--b--c|', { a: 1, b: 2, c: 3 });
    });
  });

  it('Probar Hot Observable usando Subject real', () => {
    // Versión sin TestScheduler
    const subject = new Subject<number>();
    const values1: number[] = [];
    const values2: number[] = [];

    // Primer suscriptor
    const subscription1 = subject.subscribe(val => values1.push(val));

    // Emitir valores
    subject.next(1);
    subject.next(2);

    // Segundo suscriptor (se une a mitad de camino)
    const subscription2 = subject.subscribe(val => values2.push(val));

    // Emitir más valores
    subject.next(3);
    subject.complete();

    // Verificación
    expect(values1).toEqual([1, 2, 3]);
    expect(values2).toEqual([3]); // Solo valores después de que comenzó la suscripción

    // Limpieza
    subscription1.unsubscribe();
    subscription2.unsubscribe();
  });
});
```

> [!NOTE]
> Cold Observable genera datos independientemente cada vez que se suscribe, pero Hot Observable comparte y distribuye datos.

## Mocking y Stubbing

### Mockear Servicios Dependientes

Al probar servicios que usan RxJS, es común mockear dependencias externas.

```ts
import { Observable, of } from 'rxjs';
import { map } from 'rxjs';
import { describe, it, expect, vi } from 'vitest';

type User = {
  id: number;
  name: string;
  active: boolean;
}

// Servicio bajo prueba
class UserService {
  constructor(private apiService: { fetchUsers: Function }) {}

  getUsers(): Observable<User[]> {
    return this.apiService.fetchUsers().pipe(
      map((users: User[]) => users.filter(user => user.active))
    );
  }
}

describe('Prueba de servicio', () => {
  it('Filtrar solo usuarios activos', () => {
    // Mockear servicio de API
    const mockApiService = {
      fetchUsers: vi.fn().mockReturnValue(of([
        { id: 1, name: 'Tanaka', active: true },
        { id: 2, name: 'Sato', active: false },
        { id: 3, name: 'Yamada', active: true }
      ]))
    };

    const userService = new UserService(mockApiService);
    const result$ = userService.getUsers();

    // Verificación
    result$.subscribe(users => {
      expect(users.length).toBe(2);
      expect(users[0].name).toBe('Tanaka');
      expect(users[1].name).toBe('Yamada');
      expect(mockApiService.fetchUsers).toHaveBeenCalledTimes(1);
    });
  });
});
```

### Stubs

Los stubs son objetos simples que imitan datos externos o APIs de los que depende el código bajo prueba.
Eliminan dependencias de recursos externos y permiten que las pruebas se ejecuten independientemente.
Simplemente devuelven valores fijos y no tienen lógica interna.

```ts
import { of } from 'rxjs';
import { map } from 'rxjs';
import { describe, it, expect } from 'vitest';

type User = {
  id: number;
  name: string;
  active: boolean;
};

// Servicio bajo prueba
class UserService {
  constructor(private apiService: { fetchUsers: Function }) {}

  getActiveUsers() {
    return this.apiService.fetchUsers().pipe(
      map((users: User[]) => users.filter(user => user.active))
    );
  }
}

describe('Prueba de UserService', () => {
  it('Devuelve solo usuarios activos', () => {
    // 🔹 Crear stubs
    const stubApiService = {
      fetchUsers: () => of<User[]>([
        { id: 1, name: 'Tanaka', active: true },
        { id: 2, name: 'Sato', active: false },
        { id: 3, name: 'Yamada', active: true }
      ])
    };

    // Servicio bajo prueba
    const userService = new UserService(stubApiService);

    // Verificar resultado
    userService.getActiveUsers().subscribe((users: User[]) => {
      expect(users.length).toBe(2);
      expect(users[0].name).toBe('Tanaka');
      expect(users[1].name).toBe('Yamada');
    });
  });
});
```

## Espiar suscripciones

El espía puede usarse para verificar que las suscripciones se están realizando correctamente.

```ts
import { Subject } from 'rxjs';
import { describe, it, expect, vi } from 'vitest';

describe('Prueba de suscripción', () => {
  it('Suscribirse con manejadores apropiados', () => {
    const subject = new Subject();

    // Crear espías de manejadores
    const nextSpy = vi.fn();
    const errorSpy = vi.fn();
    const completeSpy = vi.fn();

    // Suscribirse
    subject.subscribe({
      next: nextSpy,
      error: errorSpy,
      complete: completeSpy
    });

    // Emitir valores
    subject.next('value1');
    subject.next('value2');
    subject.complete();

    // Verificación
    expect(nextSpy).toHaveBeenCalledTimes(2);
    expect(nextSpy).toHaveBeenCalledWith('value1');
    expect(nextSpy).toHaveBeenCalledWith('value2');
    expect(errorSpy).not.toHaveBeenCalled();
    expect(completeSpy).toHaveBeenCalledTimes(1);
  });
});
```

## Mejores Prácticas

|Mejores prácticas|Explicación|
|---|---|
|Observar el principio de responsabilidad única| Para escribir código que se pueda probar, cada función o clase debe tener una responsabilidad única. De esta manera, las pruebas se simplifican. |
|Mockear dependencias externas|Las dependencias externas como solicitudes http y temporizadores deben mockearse y probarse en un entorno predecible. |
|Usar técnicas apropiadas para código asíncrono| Elija métodos apropiados para pruebas asíncronas, como TestScheduler, callbacks done(), o async/await. |
|Utilizar marble testing| Para probar streams complejos, use diagramas de marble para representar las expectativas de prueba de manera intuitiva.|

## Resumen

Probar código RxJS tiene aspectos que difieren del código JavaScript tradicional, como su naturaleza síncrona/asíncrona y comportamiento dependiente del tiempo. Al elegir una metodología de prueba apropiada, puede desarrollar código reactivo de alta calidad con confianza. En particular, tenga en cuenta los siguientes puntos

- Pruebas de suscripción simples para Observable síncrono
- TestScheduler o transformaciones de Promise para procesamiento asíncrono
- Marble test para código dependiente del tiempo
- Mockear dependencias externas para crear un entorno de prueba independiente.
- Diseñar código que se pueda probar según el principio de responsabilidad única

## 🔗 Secciones Relacionadas

- **[Errores Comunes y Soluciones](/es/guide/anti-patterns/common-mistakes#15-lack-of-testing)** - Verificar anti-patrones relacionados con pruebas
- **[Utilizar TestScheduler](/es/guide/testing/test-scheduler)** - Uso más detallado de TestScheduler
- **[Marble Testing](/es/guide/testing/marble-testing)** - Técnicas avanzadas de marble testing
