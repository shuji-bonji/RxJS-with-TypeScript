---
description: "Bij het testen van RxJS worden synchrone, asynchrone en tijdgebaseerde testmethoden onderscheiden. Dit artikel legt praktisch uit hoe je een robuuste teststrategie opbouwt met TestScheduler, marble testing, mocks en stubs. Inclusief implementatievoorbeelden met Jasmine/Jest en best practices voor asynchrone tests."
---

# Unit Testing van RxJS

Code geschreven met RxJS bevat veel asynchrone verwerking en vereist een andere aanpak dan traditionele testmethoden. Deze gids legt uit van basistechnieken tot geavanceerde technieken voor het effectief testen van code met RxJS.

## Testen van synchrone Observables

Laten we beginnen met het eenvoudigste geval: testen van synchrone Observables die direct voltooien.

```ts
import { Observable, of } from 'rxjs';
import { map } from 'rxjs';
import { describe, it, expect } from 'vitest';

// Te testen functie
function doubleValues(input$: Observable<number>) : Observable<number>{
  return input$.pipe(
    map(x => x * 2)
  );
}

describe('Basis Observable tests', () => {
  it('verdubbelt de waarden', () => {
    // Test Observable
    const source$ = of(1, 2, 3);
    const result$ = doubleValues(source$);

    // Verwachte resultaten
    const expected = [2, 4, 6];
    const actual: number[] = [];

    // Uitvoeren en verifiëren
    result$.subscribe({
      next: (value) => actual.push(value),
      complete: () => {
        expect(actual).toEqual(expected);
      }
    });
  });
});
```

## Testen van asynchrone Observables

Voor asynchrone Observables maken we gebruik van de asynchrone ondersteuning van het testframework.

```ts
import { Observable, timer } from 'rxjs';
import { map, take } from 'rxjs';
import { describe, it, expect } from 'vitest';

// Te testen asynchrone functie
function getDelayedValues(): Observable<number> {
  return timer(0, 100).pipe(
    map(x => x + 1),
    take(3)
  );
}

describe('Asynchrone Observable tests', () => {
  it('ontvangt asynchrone waarden in volgorde', (done: Function) => {
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

## Asynchrone tests met Promise conversie

Je kunt Observables converteren naar Promises met `firstValueFrom()` of `lastValueFrom()` en moderne JS/TS async/await gebruiken.

```ts
import { Observable, of } from 'rxjs';
import { map, delay, toArray } from 'rxjs';
import { describe, it, expect } from 'vitest';
import { lastValueFrom } from 'rxjs';

// Te testen functie
function processWithDelay(input$: Observable<number>) {
  return input$.pipe(
    map(x => x * 10),
    delay(100),
    toArray()
  );
}

describe('Tests met Promise conversie', () => {
  it('wacht op vertraagde verwerking voordat verificatie plaatsvindt', async () => {
    const source$ = of(1, 2, 3);
    const result$ = processWithDelay(source$);

    // Converteer Observable naar promise
    const result = await lastValueFrom(result$);

    // Verwachte resultaten
    expect(result).toEqual([10, 20, 30]);
  });
});
```

## Gebruik van TestScheduler

RxJS biedt een speciale scheduler genaamd `TestScheduler`, waarmee je efficiënt tijdgebaseerde operators kunt testen.

```ts
import { TestScheduler } from 'rxjs/testing';
import { map, debounceTime } from 'rxjs';
import { describe, it, beforeEach } from 'vitest';

describe('Gebruik van TestScheduler', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('test van debounceTime', () => {
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
> Marble test notatie
> Bij het gebruik van `TestScheduler` gebruik je marble diagrammen om tijdsverloop weer te geven.

## Tijd manipuleerbaar maken

Bij het testen van tijdgevoelige code (delay, debounceTime, etc.) gebruik je `TestScheduler` om de tijd te controleren.

```ts
import { TestScheduler } from 'rxjs/testing';
import { interval } from 'rxjs';
import { take, map } from 'rxjs';
import { describe, it, beforeEach } from 'vitest';

describe('Tijdcontrole', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('test met tijdversnelling', () => {
    testScheduler.run(({ expectObservable }) => {
      const source = interval(1000).pipe(
        take(3),
        map(x => x + 1)
      );

      // Duurt eigenlijk 3 seconden, maar wordt direct uitgevoerd in de testomgeving
      const expected = '1s a 999ms b 999ms (c|)';
      const values = { a: 1, b: 2, c: 3 };

      expectObservable(source).toBe(expected, values);
    });
  });
});
```

## Testen van error handling (TestScheduler versie)

Het is ook belangrijk om het gedrag van Observables bij fouten te testen.

```ts
import { TestScheduler } from 'rxjs/testing';
import { throwError, of } from 'rxjs';
import { catchError } from 'rxjs';

describe('Error handling tests', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('wanneer Observable een fout signaleert', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source = cold('  --a--b--#');
      const expected =     '--a--b--#';

      expectObservable(source).toBe(expected);
    });
  });

  it('wanneer catchError de fout opvangt en vervangt door een waarde', () => {
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

## Marble Testing

Voor het testen van complexe streams gebruik je marble diagrammen om testverwachtingen intuïtief uit te drukken.

### Hot Observable vs Cold Observable

Met TestScheduler kun je twee soorten Observables maken: hot en cold. Het is belangrijk om dit verschil te begrijpen bij het testen.

```ts
import { TestScheduler } from 'rxjs/testing';
import { Subject } from 'rxjs';
import { describe, it, beforeEach, expect } from 'vitest';

describe('Hot vs Cold Observable tests', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('Cold Observable genereert een onafhankelijke stream voor elke subscriptie', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      // Cold Observable (onafhankelijk voor elke subscriber)
      const source = cold('--a--b--c|', { a: 1, b: 2, c: 3 });

      // Eerste subscriptie
      expectObservable(source).toBe('--a--b--c|', { a: 1, b: 2, c: 3 });

      // Tweede subscriptie (start vanaf het begin)
      expectObservable(source).toBe('--a--b--c|', { a: 1, b: 2, c: 3 });
    });
  });

  it('Hot Observable deelt de stream tussen subscribers', () => {
    testScheduler.run(({ hot, expectObservable }) => {
      // Hot Observable (gedeeld tussen subscribers)
      const source = hot('--a--b--c|', { a: 1, b: 2, c: 3 });

      // Late subscriptie (ontvangt alleen waarden vanaf subscriptiemoment)
      // expectObservable(source, '-----^---').toBe('-----b--c|', { b: 2, c: 3 });
      expectObservable(source, '----^').toBe('-----b--c|', { b: 2, c: 3 });

      // Subscriptie vanaf het begin (ontvangt alle waarden)
      expectObservable(source).toBe('--a--b--c|', { a: 1, b: 2, c: 3 });
    });
  });

  it('test van Hot Observable met echte Subject', () => {
    // Niet-TestScheduler versie
    const subject = new Subject<number>();
    const values1: number[] = [];
    const values2: number[] = [];

    // Eerste subscriber
    const subscription1 = subject.subscribe(val => values1.push(val));

    // Waarden uitsturen
    subject.next(1);
    subject.next(2);

    // Tweede subscriber (halverwege)
    const subscription2 = subject.subscribe(val => values2.push(val));

    // Meer waarden uitsturen
    subject.next(3);
    subject.complete();

    // Verificatie
    expect(values1).toEqual([1, 2, 3]);
    expect(values2).toEqual([3]); // Alleen waarden na subscriptie

    // Opruimen
    subscription1.unsubscribe();
    subscription2.unsubscribe();
  });
});
```

> [!NOTE]
> Cold Observables genereren onafhankelijk data bij elke subscriptie, terwijl Hot Observables data delen en distribueren.

## Gebruik van Mocks en Stubs

### Mocken van afhankelijke services

Bij het testen van services die RxJS gebruiken, is het gebruikelijk om externe afhankelijkheden te mocken.

```ts
import { Observable, of } from 'rxjs';
import { map } from 'rxjs';
import { describe, it, expect, vi } from 'vitest';

type User = {
  id: number;
  name: string;
  active: boolean;
}

// Te testen service
class UserService {
  constructor(private apiService: { fetchUsers: Function }) {}

  getUsers(): Observable<User[]> {
    return this.apiService.fetchUsers().pipe(
      map((users: User[]) => users.filter(user => user.active))
    );
  }
}

describe('Service tests', () => {
  it('filtert alleen actieve gebruikers', () => {
    // Mock API service
    const mockApiService = {
      fetchUsers: vi.fn().mockReturnValue(of([
        { id: 1, name: 'Tanaka', active: true },
        { id: 2, name: 'Sato', active: false },
        { id: 3, name: 'Yamada', active: true }
      ]))
    };

    const userService = new UserService(mockApiService);
    const result$ = userService.getUsers();

    // Verificatie
    result$.subscribe(users => {
      expect(users.length).toBe(2);
      expect(users[0].name).toBe('Tanaka');
      expect(users[1].name).toBe('Yamada');
      expect(mockApiService.fetchUsers).toHaveBeenCalledTimes(1);
    });
  });
});
```

### Gebruik van Stubs

Een stub is een simpel object dat externe data of APIs imiteert waarvan de te testen code afhankelijk is.
Het elimineert afhankelijkheden van externe bronnen en zorgt ervoor dat tests onafhankelijk werken.
Het retourneert alleen vaste waarden en heeft geen interne logica.

```ts
import { of } from 'rxjs';
import { map } from 'rxjs';
import { describe, it, expect } from 'vitest';

type User = {
  id: number;
  name: string;
  active: boolean;
};

// Te testen service
class UserService {
  constructor(private apiService: { fetchUsers: Function }) {}

  getActiveUsers() {
    return this.apiService.fetchUsers().pipe(
      map((users: User[]) => users.filter(user => user.active))
    );
  }
}

describe('UserService tests', () => {
  it('retourneert alleen actieve gebruikers', () => {
    // 🔹 Stub maken
    const stubApiService = {
      fetchUsers: () => of<User[]>([
        { id: 1, name: 'Tanaka', active: true },
        { id: 2, name: 'Sato', active: false },
        { id: 3, name: 'Yamada', active: true }
      ])
    };

    // Te testen service
    const userService = new UserService(stubApiService);

    // Resultaat controleren
    userService.getActiveUsers().subscribe((users: User[]) => {
      expect(users.length).toBe(2);
      expect(users[0].name).toBe('Tanaka');
      expect(users[1].name).toBe('Yamada');
    });
  });
});
```

## Spies voor subscripties

Je kunt spies gebruiken om te verifiëren dat subscripties correct worden uitgevoerd.

```ts
import { Subject } from 'rxjs';
import { describe, it, expect, vi } from 'vitest';

describe('Subscriptie tests', () => {
  it('subscribeert met de juiste handlers', () => {
    const subject = new Subject();

    // Spies maken voor handlers
    const nextSpy = vi.fn();
    const errorSpy = vi.fn();
    const completeSpy = vi.fn();

    // Subscriben
    subject.subscribe({
      next: nextSpy,
      error: errorSpy,
      complete: completeSpy
    });

    // Waarden uitsturen
    subject.next('value1');
    subject.next('value2');
    subject.complete();

    // Verificatie
    expect(nextSpy).toHaveBeenCalledTimes(2);
    expect(nextSpy).toHaveBeenCalledWith('value1');
    expect(nextSpy).toHaveBeenCalledWith('value2');
    expect(errorSpy).not.toHaveBeenCalled();
    expect(completeSpy).toHaveBeenCalledTimes(1);
  });
});
```

## Best Practices

|Best Practice|Uitleg|
|---|---|
|Volg het single responsibility principe|Om testbare code te schrijven, zorg ervoor dat elke functie of klasse een enkele verantwoordelijkheid heeft. Dit maakt tests ook eenvoudiger.|
|Mock externe afhankelijkheden|Mock externe afhankelijkheden zoals HTTP requests en timers om te testen in een voorspelbare omgeving.|
|Gebruik geschikte technieken voor asynchrone code|Voor asynchrone tests kies je de juiste methode: TestScheduler, done() callback, of async/await.|
|Maak gebruik van marble testing|Voor complexe stream tests gebruik je marble diagrammen om testverwachtingen intuïtief uit te drukken.

## Samenvatting

Het testen van RxJS-code heeft verschillende aspecten die afwijken van traditionele JavaScript-code, zoals synchrone/asynchrone eigenschappen en tijdgevoelig gedrag. Door de juiste testmethoden te kiezen kun je met vertrouwen hoogwaardige reactieve code ontwikkelen. Let vooral op de volgende punten:

- Gebruik eenvoudige subscriptietests voor synchrone Observables
- Gebruik TestScheduler of Promise conversie voor asynchrone verwerking
- Gebruik marble tests voor tijdgevoelige code
- Mock externe afhankelijkheden om een geïsoleerde testomgeving te creëren
- Volg het single responsibility principe en ontwerp testbare code

## 🔗 Gerelateerde secties

- **[Veelvoorkomende fouten en oplossingen](/nl/guide/anti-patterns/common-mistakes#15-テストの欠如)** - Bekijk anti-patronen gerelateerd aan testen
- **[Gebruik van TestScheduler](/nl/guide/testing/test-scheduler)** - Meer gedetailleerd gebruik van TestScheduler
- **[Marble Testing](/nl/guide/testing/marble-testing)** - Geavanceerde marble test technieken
