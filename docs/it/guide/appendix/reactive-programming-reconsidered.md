---
description: "La Programmazione Reattiva è davvero una panacea? Esamineremo il divario tra filosofia di design e realtà, e spiegheremo oggettivamente i punti di forza e le limitazioni di RP, nonché le aree che dovrebbero essere applicate e quelle che dovrebbero essere evitate."
titleTemplate: ':title'
---

# RP Riconsiderata - Design vs Realta

La Programmazione Reattiva (RP) è ampiamente conosciuta come un paradigma potente per l'elaborazione di flussi di dati asincroni.

Ma **RP è davvero una panacea?** Questa pagina esamina il divario tra gli ideali di RP e la realtà, e considera oggettivamente dove RP dovrebbe e non dovrebbe essere usata.


## Ideali di RP vs. realtà

### Ideale: Design moderno sofisticato

RP viene spesso pubblicizzata come segue

- Codice **dichiarativo** e facile da leggere
- Espressione **concisa** dell'elaborazione asincrona
- **gestisce flussi di dati complessi** in modo unificato
- **tecnologia core dell'architettura reattiva**.

### Realtà: Può ridurre la produttività del team.

Tuttavia, i seguenti problemi sono stati riscontrati in progetti reali

- **curva di apprendimento molto alta**
- **Difficile da debuggare**
- **Testing complesso**
- **Perdita di produttività dovuta a uso improprio**

> [!WARNING]
> Applicare RP a "tutto il codice" può al contrario aumentare la complessità del codice e ridurre la produttività del team.

## Quattro sfide che affronta RP

### 1. Altezza della curva di apprendimento

Padroneggiare RP richiede un modello di pensiero diverso dalla programmazione imperativa tradizionale.

#### Difficile tracciare il flusso dati

```typescript
// ❌ Flusso dati difficile da vedere
source$
  .pipe(
    mergeMap(x => fetchData(x)),
    switchMap(data => processData(data)),
    concatMap(result => saveData(result))
  )
  .subscribe(/*...*/);
```

::: warning problema
- Le differenze tra `mergeMap`, `switchMap` e `concatMap` non sono intuitive
- Difficile tracciare dove e come i dati vengono trasformati
- Difficile identificare dove si è verificato l'errore
:::

#### Difficoltà di debugging e logging

```typescript
// Difficile da debuggare
source$
  .pipe(
    map(x => x * 2),
    filter(x => x > 10),
    mergeMap(x => api(x))
  )
  .subscribe(/*...*/);

// Dove si è verificato l'errore?
// Quale operatore ha perso il valore?
```

> [!TIP]
> L'operatore `tap()` viene usato per il debugging, che di per sé è un costo di apprendimento aggiuntivo.
> ```typescript
> source$
>   .pipe(
>     tap(x => console.log('Prima di map:', x)),
>     map(x => x * 2),
>     tap(x => console.log('Dopo map:', x)),
>     filter(x => x > 10),
>     tap(x => console.log('Dopo filter:', x))
>   )
>   .subscribe(/*...*/);
> ```

### 2. Alto carico cognitivo

Ci sono più di 100 operatori in RP, e il loro uso è complesso.

#### Troppe scelte di operatori.

| criterio | opzioni | differenza |
|------|--------|------|
| Elaborazione sequenziale di array | `concatMap`, `mergeMap`, `switchMap`, `exhaustMap` | Concorrenza e garanzie di ordine differiscono |
| Combinare Stream Multipli | `concat`, `merge`, `combineLatest`, `zip`, `forkJoin`, `race` | Metodi di combinazione diversi |
| gestione errori | `catchError`, `retry`, `retryWhen`, `onErrorResumeNext` | Strategie di retry diverse |

**Perché disturbarsi a scrivere un processo in RP che può essere fatto con un semplice `if` o `await`?**

```typescript
// ❌ Esempio di scrittura complessa in RP
of(user)
  .pipe(
    mergeMap(u => u.isPremium
      ? fetchPremiumData(u)
      : fetchBasicData(u)
    )
  )
  .subscribe(/*...*/);

// ✅ Branching condizionale semplice
const data = user.isPremium
  ? await fetchPremiumData(user)
  : await fetchBasicData(user);
```

### 3. Difficoltà dei test

Testare RP richiede una comprensione del controllo del tempo e del Marble Testing.

#### Costo di apprendimento del Marble Testing

```typescript
import { TestScheduler } from 'rxjs/testing';

it('test debounceTime', () => {
  const testScheduler = new TestScheduler((actual, expected) => {
    expect(actual).toEqual(expected);
  });

  testScheduler.run(({ cold, expectObservable }) => {
    const input$  = cold('-a-b-c---|');
    const expected =     '-----c---|';

    const result$ = input$.pipe(debounceTime(50, testScheduler));

    expectObservable(result$).toBe(expected);
  });
});
```

::: warning problema
- Bisogna imparare la notazione Marble Diagram
- Bisogna capire il meccanismo di controllo del tempo
- Costo di apprendimento più alto del normale unit testing
:::

#### Bug di sincronizzazione frequenti

```typescript
// ❌ Bug comuni: problemi di timing della sottoscrizione
const subject$ = new Subject();

subject$.next(1);  // questo valore non viene accettato
subject$.subscribe(x => console.log(x));  // sottoscrizione in ritardo
subject$.next(2);  // questo viene preso.
```

### 4. Complicazioni dovute a uso improprio

Applicare RP a tutti i codici crea complessità non necessaria.

#### Applicazione eccessiva a semplici processi CRUD

```typescript
// ❌ Applicazione RP eccessiva
getUserById(userId: string): Observable<User> {
  return this.http.get<User>(`/api/users/${userId}`)
    .pipe(
      map(user => this.transformUser(user)),
      catchError(error => {
        console.error('Errore:', error);
        return throwError(() => error);
      })
    );
}

// ✅ Promise semplice
async getUserById(userId: string): Promise<User> {
  try {
    const user = await fetch(`/api/users/${userId}`).then(r => r.json());
    return this.transformUser(user);
  } catch (error) {
    console.error('Errore:', error);
    throw error;
  }
}
```

> [!IMPORTANT]
> **RP non è un "proiettile d'argento che risolve tutti i problemi."** È importante identificare le aree da applicare e le aree da evitare.

## Aree in cui RP eccelle

RP non è una panacea, ma è molto potente nelle seguenti aree

### 1. Elaborazione continua di stream di dati

Ideale per elaborare **dati che si verificano continuamente** come dati sensori, log stream, dati in tempo reale, ecc.

```typescript
// ✅ Esempio dove RP ha un vantaggio: elaborazione dati sensori
sensorStream$
  .pipe(
    filter(reading => reading.value > threshold),
    bufferTime(1000),                           // Aggrega ogni secondo
    map(readings => calculateAverage(readings)),
    distinctUntilChanged()                      // Notifica solo quando c'è un cambiamento
  )
  .subscribe(avg => updateDashboard(avg));
```

### 2. WebSocket e notifiche push

Ideale per comunicazione bidirezionale e consegna dati di tipo push dal server.

```typescript
// ✅ Elaborazione reattiva della comunicazione WebSocket
const socket$ = webSocket('wss://example.com/socket');

socket$
  .pipe(
    retry({ count: 3, delay: 1000 }),  // Auto-riconnessione
    map(msg => parseMessage(msg)),
    filter(msg => msg.type === 'notification')
  )
  .subscribe(notification => showNotification(notification));
```

### 3. Sistema di gestione stato

NgRx, Redux Observable, MobX, ecc., sono efficaci come fondamento per librerie di gestione stato.

```typescript
// ✅ Utilizzo RP nella gestione stato (NgRx Effects)
loadUsers$ = createEffect(() =>
  this.actions$.pipe(
    ofType(UserActions.loadUsers),
    mergeMap(() =>
      this.userService.getUsers().pipe(
        map(users => UserActions.loadUsersSuccess({ users })),
        catchError(error => of(UserActions.loadUsersFailure({ error })))
      )
    )
  )
);
```

### 4. I/O non-blocking back-end

Adatto per elaborazione backend asincrona come Node.js Streams, Spring WebFlux, Vert.x, ecc.

```typescript
// ✅ Elaborazione simile a RP di Node.js Streams
const fileStream = fs.createReadStream('large-file.txt');
const transformStream = new Transform({
  transform(chunk, encoding, callback) {
    const processed = processChunk(chunk);
    callback(null, processed);
  }
});

fileStream.pipe(transformStream).pipe(outputStream);
```

### 5. Sistema distribuito event driven

Kafka, RabbitMQ, Akka Streams, ecc. sono efficaci come fondamento per architetture event-driven.

## Aree in cui RP non è adatta

Nelle seguenti aree, il codice è più semplice e manutenibile senza RP.

### 1. Elaborazione CRUD semplice

Per semplici operazioni di lettura/scrittura sul database, `async`/`await` è più adatto.

```typescript
// ❌ Non c'è bisogno di scrivere in RP
getUser(id: string): Observable<User> {
  return this.http.get<User>(`/api/users/${id}`);
}

// ✅ async/await è sufficiente
async getUser(id: string): Promise<User> {
  return await fetch(`/api/users/${id}`).then(r => r.json());
}
```

### 2. Branching condizionale semplice

Non c'è bisogno di fare stream di un processo che può essere fatto con una semplice istruzione `if`.

```typescript
// ❌ Applicazione RP eccessiva
of(value)
  .pipe(
    mergeMap(v => v > 10 ? doA(v) : doB(v))
  )
  .subscribe();

// ✅ Branching condizionale semplice
if (value > 10) {
  doA(value);
} else {
  doB(value);
}
```

### 3. Elaborazione asincrona una tantum

Se Promise è sufficiente, non c'è bisogno di renderlo Observable.

```typescript
// ❌ Observable non necessario
from(fetchData()).subscribe(data => process(data));

// ✅ Promise è abbastanza
fetchData().then(data => process(data));
```

## Evoluzione di RP: Verso astrazioni più semplici

La filosofia di RP non sta scomparendo, ma si sta **evolvendo** in una forma più semplice e trasparente.

### Angular Signals (Angular 19+)

```typescript
// Reattività basata su Signal
const count = signal(0);
const doubled = computed(() => count() * 2);

effect(() => {
  console.log('Count:', count());
});

count.set(5);  // Semplice e intuitivo
```


::: info Caratteristiche:
- Costo di apprendimento più basso di RxJS
- Facile da debuggare
- Reattività fine-grained
:::

### React Concurrent Features

```typescript
// Concurrent Rendering di React 18
function UserProfile({ userId }) {
  const user = use(fetchUser(userId));  // Integrato con Suspense
  return <div>{user.name}</div>;
}
```

::: info Caratteristiche:
- Data fetching dichiarativo
- Controllo priorità automatico
- Nasconde la complessità di RP
:::

### Svelte 5 Runes

```typescript
// Runes di Svelte 5 ($state, $derived)
let count = $state(0);
let doubled = $derived(count * 2);

function increment() {
  count++;  // Aggiornamenti intuitivi
}
```

::: info Caratteristiche:
- Ottimizzazione da compilatore
- Nessun boilerplate
- Trasparenza della reattività
:::

> [!TIP]
> Queste nuove astrazioni riducono significativamente la **complessità** pur mantenendo il **valore core** di RP (reattività).

## Policy per l'uso appropriato di RP

### 1. Determinare le aree problematiche

| Adatto per. | Non adatto per |
|-----------|-------------|
| stream di dati continuo | CRUD Semplice |
| Comunicazione WebSocket | Chiamata API una tantum |
| Integrazione eventi asincroni multipli | Branching condizionale semplice |
| elaborazione dati in tempo reale | Conversione dati statica |
| gestione stato | Aggiornamento variabile semplice |

### 2. Introduzione graduale

```typescript
// ❌ Non introdurre tutto di colpo
class UserService {
  getUser$ = (id: string) => this.http.get<User>(`/api/users/${id}`);
  updateUser$ = (user: User) => this.http.put<User>(`/api/users/${user.id}`, user);
  deleteUser$ = (id: string) => this.http.delete(`/api/users/${id}`);
  // Rendere tutto Observable
}

// ✅ RP solo dove necessario
class UserService {
  async getUser(id: string): Promise<User> { /* ... */ }
  async updateUser(user: User): Promise<User> { /* ... */ }

  // Observable solo dove sono richiesti aggiornamenti in tempo reale
  watchUser(id: string): Observable<User> {
    return this.websocket.watch(`/users/${id}`);
  }
}
```

### 3. Considerare il livello di competenza del team

| Stato del Team | Approccio Consigliato |
|------------|---------------|
| Non familiare con RP | Implementazione limitata (solo dove ci sono chiari vantaggi come WebSocket) |
| Parzialmente padroneggiato | Espansione graduale (gestione stato, elaborazione in tempo reale) |
| Tutti competenti | Utilizzo full stack (front-end al back-end) |

### 4. Confrontare con le alternative

```typescript
// Pattern 1: RP (quando devono essere integrati eventi multipli)
combineLatest([
  formValue$,
  validation$,
  apiStatus$
]).pipe(
  map(([value, isValid, status]) => ({
    canSubmit: isValid && status === 'ready',
    value
  }))
);

// Pattern 2: Signals (reattività più semplice)
const formValue = signal({});
const validation = signal(false);
const apiStatus = signal('ready');
const canSubmit = computed(() =>
  validation() && apiStatus() === 'ready'
);

// Pattern 3: async/await (elaborazione una tantum)
async function submitForm() {
  const isValid = await validateForm(formValue);
  if (!isValid) return;

  const result = await submitToApi(formValue);
  return result;
}
```

## Riepilogo

### RP non è una panacea

> [!IMPORTANT]
> La Programmazione Reattiva non è né **dannosa né una panacea**. È uno **strumento specializzato** ottimizzato per problemi asincroni e di flusso eventi.

### Apprezzare il valore di RP, ma comprendere i suoi limiti.

::: tip Aree dove RP eccelle
- Elaborazione continua stream di dati
- WebSocket e comunicazione in tempo reale
- Sistemi di gestione stato
- I/O non-blocking back-end
- Sistemi distribuiti event-driven
:::

::: warning Aree dove RP non è adatta
- Elaborazione CRUD semplice
- Branching condizionale semplice
- Elaborazione asincrona una tantum
:::

### Passaggio a una nuova astrazione

La filosofia di RP si sta evolvendo in **forme più semplici e trasparenti** come Angular Signals, React Concurrent Features e Svelte Runes.

### Linee guida per l'applicazione pratica

1. **Identificare le aree problematiche** - RP è davvero necessaria?
2. **Implementare in fasi** - non adottarla improvvisamente
3. **Considerare la competenza del team** - i costi di apprendimento sono alti
4. **Confrontare con le alternative** - async/await e Signals sono sufficienti?

> [!TIP]
> **"Usa gli strumenti giusti, nel posto giusto."** Questa è la chiave per un RP di successo.

## Pagine Correlate

- [Mappa Architettura Reattiva](/it/guide/appendix/reactive-architecture-map) - 7 livelli dove RP è attiva
- [RxJS e l'Ecosistema Reactive Streams](/it/guide/appendix/rxjs-and-reactive-streams-ecosystem) - L'intero stack tecnologico RP
- [Superare le Difficoltà di RxJS](/it/guide/overcoming-difficulties/) - Superare le barriere di apprendimento RP
- [Anti-Pattern RxJS](/it/guide/anti-patterns/) - Evitare l'uso improprio di RP

## Riferimenti

- [GitHub Discussion #17 - Reactive Programming Reconsidered](https://github.com/shuji-bonji/RxJS-with-TypeScript/discussions/17)
- [Documentazione Ufficiale Angular Signals](https://angular.dev/guide/signals)
- [React Concurrent Features](https://react.dev/blog/2022/03/29/react-v18)
- [Svelte 5 Runes](https://svelte.dev/docs/svelte/what-are-runes)
