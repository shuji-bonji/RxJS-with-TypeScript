---
description: "Een gedetailleerde uitleg van het verschil tussen Cold Observable en Hot Observable. De onafhankelijkheid van datastromen per abonnement, Hot maken met share en shareReplay, multicasting met BehaviorSubject, en type-veilige implementatiepatronen in TypeScript."
---
# Cold Observable en Hot Observable

Een van de belangrijke concepten bij het gebruik van RxJS is het onderscheid tussen "Cold Observable" en "Hot Observable". Het begrijpen van dit verschil is essentieel om efficiënt gebruik van Observable te beheersen.

## Waarom is begrip van Cold/Hot belangrijk

Als je het verschil tussen Cold/Hot niet begrijpt, loop je tegen de volgende problemen aan.

- **Onbedoelde dubbele uitvoering** - API aanroepen worden meerdere keren uitgevoerd
- **Geheugen leaks** - Abonnementen kunnen niet passend worden beheerd
- **Prestatieproblemen** - Onnodige processing wordt herhaald
- **Data-inconsistentie** - Verwachte data wordt niet ontvangen

## Verschil Cold vs Hot (vergelijkingstabel)

Laten we eerst het grote geheel begrijpen.

| Vergelijkingsitem | Cold Observable | Hot Observable |
|----------|------------------|----------------|
| **Uitvoering zonder abonnement** | Wordt niet uitgevoerd (uitgevoerd alleen bij abonnement) | Wordt uitgevoerd (stroomt waarden ook zonder subscribe) |
| **Data-uitgave timing** | Begint bij `subscribe()` | Begint op timing van uitgavezijde (onafhankelijk van abonnement) |
| **Hergebruik van uitvoering** | Elke keer opnieuw uitgevoerd | Bestaande stream wordt door meerdere gedeeld |
| **Data-consistentie** | Elk abonnement ontvangt onafhankelijke waarden | Halverwege abonneren betekent geen ontvangst van eerdere waarden |
| **Belangrijkste gebruiksvoorbeelden** | HTTP requests, asynchrone processing | UI events, WebSocket, realtime communicatie |
| **Gebruikssituatie** | Wanneer processing per afzonderlijk moet worden uitgevoerd | Status delen, event broadcasting |

**Beoordelingscriterium:** Moet processing opnieuw worden uitgevoerd voor elke abonnee? Of moet stream worden gedeeld?

## Beoordelingscriterium voor Cold vs Hot

Om daadwerkelijk te onderscheiden of een Observable Cold of Hot is, kun je aan de hand van de volgende criteria beoordelen.

| Beoordelingspunt | Cold | Hot |
|-------------|------|-----|
| **Wordt uitvoeringslogica opnieuw uitgevoerd per abonnement?** | ✅ Elke keer opnieuw | ❌ Uitvoering gedeeld |
| **Stromen data voor abonnement?** | ❌ Wacht tot abonnement | ✅ Stroomt onafhankelijk van abonnement |
| **Ontvangen meerdere abonnementen dezelfde data?** | ❌ Onafhankelijke data | ✅ Delen dezelfde data |

### Praktische herkenningswijze

Je kunt eenvoudig beoordelen met de volgende test.

```typescript
const observable$ = /* Observable om te onderzoeken */;

observable$.subscribe(/* abonnement 1 */);
observable$.subscribe(/* abonnement 2 */);

// ✅ Cold: console.log binnen Observable wordt 2 keer uitgevoerd
//         (uitvoeringslogica wordt opnieuw uitgevoerd per abonnement)
// ✅ Hot:  console.log binnen Observable wordt 1 keer uitgevoerd
//         (uitvoering wordt gedeeld)
```

**Concreet voorbeeld:**

```typescript
import { Observable, Subject } from 'rxjs';

// Cold Observable
const cold$ = new Observable(subscriber => {
  console.log('Cold: Uitvoering start');
  subscriber.next(Math.random());
});

cold$.subscribe(v => console.log('Abonnement 1:', v));
cold$.subscribe(v => console.log('Abonnement 2:', v));
// Output:
// Cold: Uitvoering start  ← 1e keer
// Abonnement 1: 0.123...
// Cold: Uitvoering start  ← 2e keer (wordt opnieuw uitgevoerd)
// Abonnement 2: 0.456...

// Hot Observable
const hot$ = new Subject();

hot$.subscribe(v => console.log('Abonnement 1:', v));
hot$.subscribe(v => console.log('Abonnement 2:', v));
hot$.next(1); // Data-uitgifte slechts 1 keer
// Output:
// Abonnement 1: 1
// Abonnement 2: 1  ← Delen dezelfde data
```

## Cold/Hot classificatietabel per Creation Function

Voor alle belangrijke Creation Functions classificeren we Cold/Hot. Hierdoor zie je in één oogopslag welke functie welke Observable genereert.

| Categorie | Creation Function | Cold/Hot | Opmerking |
|---------|-------------------|----------|------|
| **Basiscreatie** | `of()` | ❄️ Cold | Geeft waarden opnieuw uit per abonnement |
| | `from()` | ❄️ Cold | Voert array/Promise opnieuw uit per abonnement |
| | `fromEvent()` | ❄️ Cold | Voegt onafhankelijke listener toe per abonnement [^fromEvent] |
| | `interval()` | ❄️ Cold | Onafhankelijke timer per abonnement |
| | `timer()` | ❄️ Cold | Onafhankelijke timer per abonnement |
| **Loop generatie** | `range()` | ❄️ Cold | Regenereert bereik per abonnement |
| | `generate()` | ❄️ Cold | Voert loop opnieuw uit per abonnement |
| **HTTP communicatie** | `ajax()` | ❄️ Cold | Nieuwe HTTP request per abonnement |
| | `fromFetch()` | ❄️ Cold | Nieuwe Fetch request per abonnement |
| **Combinatie** | `concat()` | ❄️ Cold | Erft karakter van originele Observable [^combination] |
| | `merge()` | ❄️ Cold | Erft karakter van originele Observable [^combination] |
| | `combineLatest()` | ❄️ Cold | Erft karakter van originele Observable [^combination] |
| | `zip()` | ❄️ Cold | Erft karakter van originele Observable [^combination] |
| | `forkJoin()` | ❄️ Cold | Erft karakter van originele Observable [^combination] |
| **Selectie/partitie** | `race()` | ❄️ Cold | Erft karakter van originele Observable [^combination] |
| | `partition()` | ❄️ Cold | Erft karakter van originele Observable [^combination] |
| **Conditionele vertakking** | `iif()` | ❄️ Cold | Erft karakter van conditioneel geselecteerde Observable |
| | `defer()` | ❄️ Cold | Voert factory functie uit per abonnement |
| **Controle** | `scheduled()` | ❄️ Cold | Erft karakter van originele Observable |
| | `using()` | ❄️ Cold | Maakt resource per abonnement |
| **Subject familie** | `new Subject()` | 🔥 Hot | Altijd Hot |
| | `new BehaviorSubject()` | 🔥 Hot | Altijd Hot |
| | `new ReplaySubject()` | 🔥 Hot | Altijd Hot |
| | `new AsyncSubject()` | 🔥 Hot | Altijd Hot |
| **WebSocket** | `webSocket()` | 🔥 Hot | Deelt WebSocket-verbinding |

[^fromEvent]: `fromEvent()` is Cold omdat het per abonnement een onafhankelijke event listener toevoegt. Echter, het event zelf vindt plaats onafhankelijk van abonnement, dus wordt het vaak verward met Hot.

[^combination]: Combinatie Creation Functions zijn Cold als de originele Observables Cold zijn, Hot als ze Hot zijn. Normaal worden vaak Cold Observables gecombineerd.

> [!IMPORTANT] Belangrijk principe
> **Bijna alle Creation Functions genereren Cold.**
> Alleen de volgende genereren Hot:
> - Subject familie (Subject, BehaviorSubject, ReplaySubject, AsyncSubject)
> - webSocket()

## Cold Observable

### Kenmerken

- **Bij elk abonnement wordt een nieuwe datastroom aangemaakt**
- **Begint data-uitgifte niet tot abonnement (uitgestelde uitvoering)**
- **Alle abonnees ontvangen alle data vanaf het begin van Observable**

Bij Cold Observable wordt bij elk subscribe een nieuwe uitvoeringscontext aangemaakt.
Dit is geschikt voor HTTP requests en asynchrone processing waar elke keer nieuwe processing nodig is.

### Codevoorbeeld

```typescript
import { Observable } from 'rxjs';

// Voorbeeld van Cold Observable
const cold$ = new Observable<number>(subscriber => {
  console.log('Aanmaak databron - Nieuw abonnement');
  const randomValue = Math.random();
  subscriber.next(randomValue);
  subscriber.complete();
});

// 1e abonnement
console.log('--- 1e abonnement ---');
cold$.subscribe(value => console.log('Abonnee 1:', value));

// 2e abonnement (verschillende data wordt gegenereerd)
console.log('--- 2e abonnement ---');
cold$.subscribe(value => console.log('Abonnee 2:', value));
```

#### Uitvoeringsresultaat
```sh
--- 1e abonnement ---
Aanmaak databron - Nieuw abonnement
Abonnee 1: 0.259632...
--- 2e abonnement ---
Aanmaak databron - Nieuw abonnement  ← Wordt opnieuw uitgevoerd
Abonnee 2: 0.744322...  ← Andere waarde
```

> [!TIP] Belangrijk punt
> Bij elk abonnement wordt "Aanmaak databron" uitgevoerd en verschillende waarden gegenereerd.

### Veelvoorkomende Cold Observables (herkenningswijze)

De volgende Observables zijn normaal Cold.

```typescript
import { of, from, interval, timer } from 'rxjs';
import { ajax } from 'rxjs/ajax';

// Creation Functions
of(1, 2, 3)                    // Cold
from([1, 2, 3])                // Cold
from(fetch('/api/data'))       // Cold

// Tijd operators
interval(1000)                 // Cold
timer(1000)                    // Cold

// HTTP requests
ajax('/api/users')             // Cold
```

> [!TIP] Regel
> Creation Functions, tijd operators, HTTP requests zijn in principe Cold

## Hot Observable

### Kenmerken

- **Stroomt waarden ook zonder subscribe (wordt uitgevoerd onafhankelijk van aanwezigheid abonnement)**
- **Ontvangt alleen data vanaf het moment van abonnement**
- **Eén databron wordt gedeeld door meerdere abonnees**

Bij Hot Observable is de timing van stream-uitgifte onafhankelijk van abonnement, en abonnees nemen halverwege deel.

### Codevoorbeeld

```typescript
import { Subject } from 'rxjs';

// Voorbeeld van Hot Observable (met Subject)
const hot$ = new Subject<number>();

// Eerste abonnement
console.log('--- Abonnee 1 start ---');
hot$.subscribe(value => console.log('Abonnee 1:', value));

// Data-uitgifte
hot$.next(1);
hot$.next(2);

// Tweede abonnement (later abonnement)
console.log('--- Abonnee 2 start ---');
hot$.subscribe(value => console.log('Abonnee 2:', value));

// Verdere data-uitgifte
hot$.next(3);
hot$.next(4);

hot$.complete();
```

#### Uitvoeringsresultaat
```sh
--- Abonnee 1 start ---
Abonnee 1: 1
Abonnee 1: 2
--- Abonnee 2 start ---
Abonnee 1: 3
Abonnee 2: 3  ← Abonnement 2 neemt deel vanaf 3 (ontvangt 1, 2 niet)
Abonnee 1: 4
Abonnee 2: 4
```

> [!TIP] Belangrijk punt
> Abonnee 2 neemt halverwege deel, dus ontvangt eerdere waarden (1, 2) niet.

### Veelvoorkomende Hot Observables (herkenningswijze)

De volgende Observables zijn altijd Hot.

```typescript
import { Subject, BehaviorSubject, ReplaySubject } from 'rxjs';
import { webSocket } from 'rxjs/webSocket';

// Subject familie (altijd Hot)
new Subject()                  // Hot
new BehaviorSubject(0)         // Hot
new ReplaySubject(1)           // Hot

// WebSocket (altijd Hot)
webSocket('ws://localhost:8080') // Hot
```

> [!TIP] Regel
> **Alleen Subject familie en webSocket() genereren Hot**

> [!WARNING] fromEvent() is Cold
> `fromEvent(button, 'click')` wordt vaak verward met Hot, maar is eigenlijk **Cold**. Het voegt per abonnement een onafhankelijke event listener toe. Het event zelf vindt plaats onafhankelijk van abonnement, maar elke abonnee heeft een onafhankelijke listener.

## Hoe Cold Observable naar Hot te converteren

RxJS biedt hoofdzakelijk de volgende methoden om Cold Observable naar Hot te converteren.

- `share()` - Eenvoudig Hot maken (aanbevolen)
- `shareReplay()` - Hot maken met caching van eerdere waarden
- ~~`multicast()`~~ - Verouderd (deprecated in RxJS v7, verwijderd in v8)

### share() operator

`share()` is de meest gebruikelijke manier om Cold Observable naar Hot Observable te converteren.

```typescript
import { interval } from 'rxjs';
import { share, take } from 'rxjs';

// Simuleren van HTTP aanroep
const makeHttpRequest = () => {
  console.log('HTTP aanroep uitgevoerd!');
  return interval(1000).pipe(take(3));
};

// ❌ Cold Observable (geen delen)
const cold$ = makeHttpRequest();

cold$.subscribe(val => console.log('Abonnee 1:', val));
cold$.subscribe(val => console.log('Abonnee 2:', val));
// → HTTP aanroep wordt 2 keer uitgevoerd

// ✅ Hot Observable (met share)
const shared$ = makeHttpRequest().pipe(share());

shared$.subscribe(val => console.log('Gedeelde abonnee 1:', val));
shared$.subscribe(val => console.log('Gedeelde abonnee 2:', val));
// → HTTP aanroep slechts 1 keer, resultaat wordt gedeeld
```

**Uitvoeringsresultaat (Cold):**
```sh
HTTP aanroep uitgevoerd!  ← 1e keer
Abonnee 1: 0
HTTP aanroep uitgevoerd!  ← 2e keer (duplicaat!)
Abonnee 2: 0
...
```

**Uitvoeringsresultaat (Hot):**
```sh
HTTP aanroep uitgevoerd!  ← Slechts 1 keer
Gedeelde abonnee 1: 0
Gedeelde abonnee 2: 0  ← Deelt dezelfde stream
...
```

> [!NOTE] Use case
> - Meerdere componenten gebruiken hetzelfde API-resultaat
> - Voorkomen van duplicatie van bijeffecten (HTTP aanroepen etc.)

### shareReplay() operator

`shareReplay()` is een uitgebreide versie van `share()` die **eerdere waarden cachet** en hergebruikt voor nieuwe abonnees.

```typescript
import { interval } from 'rxjs';
import { shareReplay, take } from 'rxjs';

const request$ = interval(1000).pipe(
  take(3),
  shareReplay(2)  // Cachet laatste 2 waarden
);

// 1e abonnement
request$.subscribe(val => console.log('Abonnee 1:', val));

// 3.5 seconden later 2e abonnement (na voltooiing stream)
setTimeout(() => {
  console.log('--- Abonnee 2 start (na voltooiing) ---');
  request$.subscribe(val => console.log('Abonnee 2:', val));
}, 3500);
```

#### Uitvoeringsresultaat
```sh
Abonnee 1: 0
Abonnee 1: 1
Abonnee 1: 2
--- Abonnee 2 start (na voltooiing) ---
Abonnee 2: 1  ← Gecachete waarde (laatste 2)
Abonnee 2: 2  ← Gecachete waarde
```

> [!NOTE] Use case
> - Cachen van API-resultaten
> - Delen van beginstatus (alleen laatste 1 item cachen)
> - Leveren van eerdere data aan vertraagde abonnees

> [!WARNING] Aandachtspunt shareReplay
> `shareReplay()` behoudt cache zelfs wanneer abonnementen 0 worden, wat geheugen leaks kan veroorzaken. Zie [Chapter 10: Misbruik van shareReplay](/nl/guide/anti-patterns/common-mistakes#4-sharereplay-の誤用) voor details.

### Over multicast()

> [!NOTE]
> `multicast()` is flexibel, maar is deprecated in RxJS v7 en verwijderd in v8. Gebruik nu `share()` of `shareReplay()`. Zie [share() operator uitleg](/nl/guide/operators/multicasting/share) voor details.

## Praktisch voorbeeld: API Cache Service

Een veelvoorkomend patroon in echte applicaties: wanneer meerdere componenten dezelfde API-data nodig hebben.

```typescript
import { Observable, of, throwError } from 'rxjs';
import { catchError, shareReplay, delay, tap } from 'rxjs';

// Eenvoudige cache service
class UserService {
  private cache$: Observable<User[]> | null = null;

  getUsers(): Observable<User[]> {
    // Als cache bestaat, retourneer die
    if (this.cache$) {
      console.log('Retourneren uit cache');
      return this.cache$;
    }

    // Maak nieuwe request en cache het
    console.log('Nieuwe request uitvoeren');
    this.cache$ = this.fetchUsersFromAPI().pipe(
      catchError(err => {
        this.cache$ = null;  // Wis cache bij fout
        return throwError(() => err);
      }),
      shareReplay(1)  // Cache laatste resultaat
    );

    return this.cache$;
  }

  private fetchUsersFromAPI(): Observable<User[]> {
    // Simuleren van daadwerkelijk API request
    return of([
      { id: 1, name: 'Taro Yamada' },
      { id: 2, name: 'Hanako Sato' }
    ]).pipe(
      delay(1000),
      tap(() => console.log('Data ontvangen van API'))
    );
  }

  clearCache(): void {
    this.cache$ = null;
    console.log('Cache gewist');
  }
}

interface User {
  id: number;
  name: string;
}

// Gebruiksvoorbeeld
const userService = new UserService();

// Component 1: Data opvragen
userService.getUsers().subscribe(users =>
  console.log('Component 1:', users)
);

// Component 2: 2 seconden later data opvragen
setTimeout(() => {
  userService.getUsers().subscribe(users =>
    console.log('Component 2:', users)
  );
}, 2000);

// Cache wissen en opnieuw opvragen
setTimeout(() => {
  userService.clearCache();
  userService.getUsers().subscribe(users =>
    console.log('Component 3:', users)
  );
}, 4000);
```

#### Uitvoeringsresultaat
```sh
Nieuwe request uitvoeren
Data ontvangen van API
Component 1: [{id: 1, name: 'Taro Yamada'}, {id: 2, name: 'Hanako Sato'}]
Retourneren uit cache  ← Geen API aanroep
Component 2: [{id: 1, name: 'Taro Yamada'}, {id: 2, name: 'Hanako Sato'}]
Cache gewist
Nieuwe request uitvoeren  ← Opnieuw API aanroep
Data ontvangen van API
Component 3: [{id: 1, name: 'Taro Yamada'}, {id: 2, name: 'Hanako Sato'}]
```

**Punten:**
- Cache laatste response met `shareReplay(1)`
- Meerdere componenten delen data (API aanroep slechts 1 keer)
- Passend weggooien van cache bij fout of wissen

## Wanneer te gebruiken

<div class="comparison-cards">

::: tip Cold
#### Wanneer te gebruiken
- Wanneer elke abonnee zijn eigen dataset nodig heeft
- Om een nieuw proces of actie weer te geven
- Wanneer duplicatie van bijeffecten geen probleem is

#### Voorbeelden
- Nieuwe POST request verzenden bij elke formulierverzending
- Verschillende timer nodig per gebruiker
- Onafhankelijke berekening uitvoeren per abonnement
:::

::: tip Hot
#### Wanneer te gebruiken
- Data delen tussen meerdere componenten
- Resources willen besparen (bijv: aantal HTTP aanroepen verminderen)
- Event stream weergeven
- Status management of communicatie tussen services

#### Voorbeelden
- Configuratie-informatie gedeeld door hele applicatie
- Inlogstatus van gebruiker
- Realtime berichten (WebSocket)
- DOM events (klik, scroll etc.)
:::

</div>

## Samenvatting

Het begrijpen van Cold Observable en Hot Observable en ze passend gebruiken is een belangrijke vaardigheid voor het bouwen van efficiënte RxJS-applicaties.

::: tip Belangrijke punten
- **Cold Observable**: Stream die begint te bewegen bij abonnement (onafhankelijke uitvoering per abonnement)
- **Hot Observable**: Delen van reeds bewegende stream (zelfde uitvoering bij meerdere abonnementen)
- **share()**: Eenvoudigste manier om Cold naar Hot te converteren
- **shareReplay()**: Naar Hot converteren met cachen van eerdere waarden (handig voor delen van API-resultaten)
:::

::: tip Ontwerpbeslissings criteria
- Is het nodig om data te delen tussen meerdere abonnees?
- Is het nodig om eerdere waarden te cachen en aan nieuwe abonnees te leveren?
- Hoe bijeffecten (HTTP requests etc.) te beheren?
:::

Op basis van deze overwegingen kun je, door passende Observable types en operators te kiezen, efficiënte en robuuste reactieve applicaties bouwen.

## Gerelateerde secties

- **[share() operator](/nl/guide/operators/multicasting/share)** - Gedetailleerde uitleg van share()
- **[Misbruik van shareReplay](/nl/guide/anti-patterns/common-mistakes#4-sharereplay-の誤用)** - Veelvoorkomende fouten en oplossingen
- **[Subject](/nl/guide/subjects/what-is-subject)** - Begrip van Hot Subject

<style scoped>
.comparison-cards {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 1rem;
  margin-bottom: 2rem;
}

@media (max-width: 768px) {
  .comparison-cards {
    grid-template-columns: 1fr;
  }
}
</style>
