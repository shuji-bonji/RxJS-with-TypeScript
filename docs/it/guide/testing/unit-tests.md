---
description: "Test unitari RxJS: tecniche per sincronizzazione, asincronia e controllo tempo. TestScheduler, marble testing, mock e stub per strategie robuste."
---

# Test Unitari di RxJS

Il codice che utilizza RxJS contiene molte operazioni asincrone e richiede un approccio diverso rispetto ai metodi di test tradizionali. Questa guida spiega le tecniche fondamentali e avanzate per testare efficacemente il codice che utilizza RxJS.

## Test di Observable Sincroni

Iniziamo dal caso più semplice: il test di Observable che si completano in modo sincrono.

```ts
import { Observable, of } from 'rxjs';
import { map } from 'rxjs';
import { describe, it, expect } from 'vitest';

// Funzione da testare
function doubleValues(input$: Observable<number>) : Observable<number>{
  return input$.pipe(
    map(x => x * 2)
  );
}

describe('Test di Observable di base', () => {
  it('raddoppia i valori', () => {
    // Observable per il test
    const source$ = of(1, 2, 3);
    const result$ = doubleValues(source$);

    // Risultato atteso
    const expected = [2, 4, 6];
    const actual: number[] = [];

    // Esecuzione e verifica
    result$.subscribe({
      next: (value) => actual.push(value),
      complete: () => {
        expect(actual).toEqual(expected);
      }
    });
  });
});
```

## Come Testare Observable Asincroni

Per gli Observable asincroni, utilizziamo il supporto asincrono del framework di test.

```ts
import { Observable, timer } from 'rxjs';
import { map, take } from 'rxjs';
import { describe, it, expect } from 'vitest';

// Funzione asincrona da testare
function getDelayedValues(): Observable<number> {
  return timer(0, 100).pipe(
    map(x => x + 1),
    take(3)
  );
}

describe('Test di Observable asincroni', () => {
  it('riceve valori asincroni in sequenza', (done: Function) => {
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

## Test Asincroni tramite Conversione in Promise

È possibile convertire gli Observable in Promise usando `firstValueFrom()` o `lastValueFrom()` e utilizzare async/await di JavaScript/TypeScript moderno.

```ts
import { Observable, of } from 'rxjs';
import { map, delay, toArray } from 'rxjs';
import { describe, it, expect } from 'vitest';
import { lastValueFrom } from 'rxjs';

// Funzione da testare
function processWithDelay(input$: Observable<number>) {
  return input$.pipe(
    map(x => x * 10),
    delay(100),
    toArray()
  );
}

describe('Test usando conversione in Promise', () => {
  it('attende l\'elaborazione con ritardo prima della verifica', async () => {
    const source$ = of(1, 2, 3);
    const result$ = processWithDelay(source$);

    // Converte Observable in promise
    const result = await lastValueFrom(result$);

    // Risultato atteso
    expect(result).toEqual([10, 20, 30]);
  });
});
```

## Utilizzo di TestScheduler

RxJS fornisce uno scheduler speciale chiamato `TestScheduler` che permette di testare efficientemente gli operatori basati sul tempo.

```ts
import { TestScheduler } from 'rxjs/testing';
import { map, debounceTime } from 'rxjs';
import { describe, it, beforeEach } from 'vitest';

describe('Utilizzo di TestScheduler', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('test di debounceTime', () => {
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
> Notazione Marble Testing
> Quando si usa `TestScheduler`, si utilizza la notazione dei diagrammi marble per rappresentare il passare del tempo.

## Rendere il Tempo Manipolabile

Quando si testano codici dipendenti dal tempo (delay, debounceTime, ecc.), si usa `TestScheduler` per controllare il tempo.

```ts
import { TestScheduler } from 'rxjs/testing';
import { interval } from 'rxjs';
import { take, map } from 'rxjs';
import { describe, it, beforeEach } from 'vitest';

describe('Controllo del tempo', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('test con avanzamento rapido del tempo', () => {
    testScheduler.run(({ expectObservable }) => {
      const source = interval(1000).pipe(
        take(3),
        map(x => x + 1)
      );

      // Richiede 3 secondi in realtà, ma viene eseguito istantaneamente nell'ambiente di test
      const expected = '1s a 999ms b 999ms (c|)';
      const values = { a: 1, b: 2, c: 3 };

      expectObservable(source).toBe(expected, values);
    });
  });
});
```

## Test di Gestione degli Errori (versione TestScheduler)

È importante anche testare il comportamento degli Observable quando si verificano errori.

```ts
import { TestScheduler } from 'rxjs/testing';
import { throwError, of } from 'rxjs';
import { catchError } from 'rxjs';

describe('Test di gestione degli errori', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('quando Observable notifica un errore', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source = cold('  --a--b--#');
      const expected =     '--a--b--#';

      expectObservable(source).toBe(expected);
    });
  });

  it('quando catchError cattura l\'errore e lo sostituisce con un valore', () => {
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

Per testare stream complessi, si esprimono i valori attesi in modo intuitivo usando i diagrammi marble.

### Hot Observable vs Cold Observable

Con TestScheduler è possibile creare due tipi di Observable: hot e cold. È importante comprendere questa differenza per i test.

```ts
import { TestScheduler } from 'rxjs/testing';
import { Subject } from 'rxjs';
import { describe, it, beforeEach, expect } from 'vitest';

describe('Test Hot vs Cold Observable', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('Cold Observable genera uno stream indipendente per ogni sottoscrizione', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      // Cold Observable (indipendente per ogni sottoscrittore)
      const source = cold('--a--b--c|', { a: 1, b: 2, c: 3 });

      // Prima sottoscrizione
      expectObservable(source).toBe('--a--b--c|', { a: 1, b: 2, c: 3 });

      // Seconda sottoscrizione (inizia dall'inizio)
      expectObservable(source).toBe('--a--b--c|', { a: 1, b: 2, c: 3 });
    });
  });

  it('Hot Observable condivide lo stream tra i sottoscrittori', () => {
    testScheduler.run(({ hot, expectObservable }) => {
      // Hot Observable (condiviso tra i sottoscrittori)
      const source = hot('--a--b--c|', { a: 1, b: 2, c: 3 });

      // Sottoscrizione ritardata (riceve solo i valori dopo l'inizio della sottoscrizione)
      // expectObservable(source, '-----^---').toBe('-----b--c|', { b: 2, c: 3 });
      expectObservable(source, '----^').toBe('-----b--c|', { b: 2, c: 3 });

      // Sottoscrizione dall'inizio (riceve tutti i valori)
      expectObservable(source).toBe('--a--b--c|', { a: 1, b: 2, c: 3 });
    });
  });

  it('test di Hot Observable usando Subject reale', () => {
    // Versione non-TestScheduler
    const subject = new Subject<number>();
    const values1: number[] = [];
    const values2: number[] = [];

    // Primo sottoscrittore
    const subscription1 = subject.subscribe(val => values1.push(val));

    // Emissione di valori
    subject.next(1);
    subject.next(2);

    // Secondo sottoscrittore (a metà strada)
    const subscription2 = subject.subscribe(val => values2.push(val));

    // Ulteriore emissione di valori
    subject.next(3);
    subject.complete();

    // Verifica
    expect(values1).toEqual([1, 2, 3]);
    expect(values2).toEqual([3]); // Solo i valori dopo l'inizio della sottoscrizione

    // Pulizia
    subscription1.unsubscribe();
    subscription2.unsubscribe();
  });
});
```

> [!NOTE]
> I Cold Observable generano dati in modo indipendente ogni volta che vengono sottoscritti, mentre gli Hot Observable condividono e distribuiscono i dati.

## Utilizzo di Mock e Stub

### Mock di Servizi Dipendenti

Quando si testano servizi che utilizzano RxJS, si mockano spesso le dipendenze esterne.

```ts
import { Observable, of } from 'rxjs';
import { map } from 'rxjs';
import { describe, it, expect, vi } from 'vitest';

type User = {
  id: number;
  name: string;
  active: boolean;
}

// Servizio da testare
class UserService {
  constructor(private apiService: { fetchUsers: Function }) {}

  getUsers(): Observable<User[]> {
    return this.apiService.fetchUsers().pipe(
      map((users: User[]) => users.filter(user => user.active))
    );
  }
}

describe('Test di servizio', () => {
  it('filtra solo gli utenti attivi', () => {
    // Mock del servizio API
    const mockApiService = {
      fetchUsers: vi.fn().mockReturnValue(of([
        { id: 1, name: 'Tanaka', active: true },
        { id: 2, name: 'Sato', active: false },
        { id: 3, name: 'Yamada', active: true }
      ]))
    };

    const userService = new UserService(mockApiService);
    const result$ = userService.getUsers();

    // Verifica
    result$.subscribe(users => {
      expect(users.length).toBe(2);
      expect(users[0].name).toBe('Tanaka');
      expect(users[1].name).toBe('Yamada');
      expect(mockApiService.fetchUsers).toHaveBeenCalledTimes(1);
    });
  });
});
```

### Utilizzo di Stub

Gli stub sono oggetti semplici che imitano dati esterni o API da cui dipende il codice da testare.
Eliminano la dipendenza da risorse esterne e permettono ai test di funzionare in modo indipendente.
Restituiscono solo valori fissi, senza logica interna.

```ts
import { of } from 'rxjs';
import { map } from 'rxjs';
import { describe, it, expect } from 'vitest';

type User = {
  id: number;
  name: string;
  active: boolean;
};

// Servizio da testare
class UserService {
  constructor(private apiService: { fetchUsers: Function }) {}

  getActiveUsers() {
    return this.apiService.fetchUsers().pipe(
      map((users: User[]) => users.filter(user => user.active))
    );
  }
}

describe('Test di UserService', () => {
  it('restituisce solo utenti attivi', () => {
    // 🔹 Creazione dello stub
    const stubApiService = {
      fetchUsers: () => of<User[]>([
        { id: 1, name: 'Tanaka', active: true },
        { id: 2, name: 'Sato', active: false },
        { id: 3, name: 'Yamada', active: true }
      ])
    };

    // Servizio da testare
    const userService = new UserService(stubApiService);

    // Verifica del risultato
    userService.getActiveUsers().subscribe((users: User[]) => {
      expect(users.length).toBe(2);
      expect(users[0].name).toBe('Tanaka');
      expect(users[1].name).toBe('Yamada');
    });
  });
});
```

## Spy sulle Sottoscrizioni

Si possono usare spy per verificare che le sottoscrizioni vengano effettuate correttamente.

```ts
import { Subject } from 'rxjs';
import { describe, it, expect, vi } from 'vitest';

describe('Test di sottoscrizione', () => {
  it('sottoscrizione con i gestori appropriati', () => {
    const subject = new Subject();

    // Creazione di spy per i gestori
    const nextSpy = vi.fn();
    const errorSpy = vi.fn();
    const completeSpy = vi.fn();

    // Sottoscrizione
    subject.subscribe({
      next: nextSpy,
      error: errorSpy,
      complete: completeSpy
    });

    // Emissione di valori
    subject.next('value1');
    subject.next('value2');
    subject.complete();

    // Verifica
    expect(nextSpy).toHaveBeenCalledTimes(2);
    expect(nextSpy).toHaveBeenCalledWith('value1');
    expect(nextSpy).toHaveBeenCalledWith('value2');
    expect(errorSpy).not.toHaveBeenCalled();
    expect(completeSpy).toHaveBeenCalledTimes(1);
  });
});
```

## Best Practice

|Best Practice|Descrizione|
|---|---|
|Rispettare il principio di singola responsabilità|Per scrivere codice testabile, assicurarsi che ogni funzione o classe abbia una singola responsabilità. Questo rende anche i test più semplici.|
|Mockare le dipendenze esterne|Mockare le dipendenze esterne come richieste HTTP o timer per testare in un ambiente prevedibile.|
|Usare tecniche appropriate per il codice asincrono|Per i test asincroni, scegliere il metodo appropriato tra TestScheduler, callback done(), o async/await.|
|Utilizzare il marble testing|Per testare stream complessi, esprimere i valori attesi in modo intuitivo usando i diagrammi marble.|

## Riepilogo

I test del codice RxJS presentano aspetti diversi dal codice JavaScript tradizionale, come la natura sincrona/asincrona e il comportamento dipendente dal tempo. Scegliendo le tecniche di test appropriate, è possibile sviluppare con sicurezza codice reattivo di alta qualità. In particolare, tenere a mente i seguenti punti:

- Per Observable sincroni, test di sottoscrizione semplici
- Per elaborazioni asincrone, TestScheduler o conversione in Promise
- Per codice dipendente dal tempo, marble testing
- Mockare le dipendenze esterne per creare un ambiente di test indipendente
- Seguire il principio di singola responsabilità e progettare codice facilmente testabile

## 🔗 Sezioni Correlate

- **[Errori Comuni e Contromisure](/it/guide/anti-patterns/common-mistakes)** - Verifica gli anti-pattern relativi ai test
- **[Utilizzo di TestScheduler](/it/guide/testing/test-scheduler)** - Utilizzo più dettagliato di TestScheduler
- **[Marble Testing](/it/guide/testing/marble-testing)** - Tecniche avanzate di marble testing
