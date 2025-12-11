---
description: "Implementatiemethode van multicasting met share() operator uitgelegd. Deel dezelfde Observable met meerdere subscribers en verminder duplicaat verwerking (API aanroepen, berekeningen). Verschillen met shareReplay(), Cold/Hot conversie en type-veilige implementatie in TypeScript worden geïntroduceerd."
---

# share - Observable Delen met Meerdere Subscribers

De `share()` operator is de eenvoudigste operator om multicasting in RxJS te implementeren.
Door meerdere subscribers dezelfde gegevensbron te laten delen, kunt u duplicaat verwerking (API verzoeken, rekenverwerking, etc.) verminderen.

[📘 RxJS Officiële Documentatie - `share()`](https://rxjs.dev/api/index/function/share)

## 🔰 Basis Gebruik

```typescript
import { interval, share, take, tap } from 'rxjs';

// Observable die telt met interval
const source$ = interval(1000).pipe(
  take(5),
  tap(value => console.log(`Bron: ${value}`)),
  share() // Multicast inschakelen
);

// Eerste subscriber
console.log('Observer 1 subscriptie starten');
const subscription1 = source$.subscribe(value =>
  console.log(`Observer 1: ${value}`)
);

// 2,5 seconden later tweede subscriber toevoegen
setTimeout(() => {
  console.log('Observer 2 subscriptie starten');
  source$.subscribe(value =>
    console.log(`Observer 2: ${value}`)
  );

  // 2,5 seconden later subscriber 1 afmelden
  setTimeout(() => {
    console.log('Observer 1 subscriptie beëindigen');
    subscription1.unsubscribe();
  }, 2500);
}, 2500);
```

### Uitvoeringsresultaat

```
Observer 1 subscriptie starten
Bron: 0
Observer 1: 0
Bron: 1
Observer 1: 1
Observer 2 subscriptie starten
Bron: 2
Observer 1: 2
Observer 2: 2
Bron: 3
Observer 1: 3
Observer 2: 3
Observer 1 subscriptie beëindigen
Bron: 4
Observer 2: 4
```

**Belangrijke Punten**:
- Bronverwerking (`tap`) wordt slechts één keer uitgevoerd
- Alle subscribers ontvangen dezelfde waarden
- Tussentijds deelnemende subscribers ontvangen alleen waarden na deelname

## 💡 Mechanisme van share()

`share()` is de standaard multicasting operator van RxJS. Intern gebruikt het Subject om naar meerdere subscribers te broadcasten.

> [!NOTE]
> **Wijzigingen in RxJS v7 en later**: Voorheen werd het uitgelegd als een combinatie van `multicast()` en `refCount()`, maar deze operators zijn verouderd in v7 en verwijderd in v8. Momenteel is `share()` de standaard multicasting methode. Voor details zie [RxJS Officiële Documentatie - Multicasting](https://rxjs.dev/deprecations/multicasting).

**Werkingsstroom**:
- **Bij eerste subscriptie**: Start verbinding met bron Observable en creëert intern Subject
- **Subscribers toevoegen**: Deel bestaande verbinding (broadcast waarden via Subject)
- **Alle subscribers afmelden**: Verbreek verbinding met bron (`resetOnRefCountZero: true` geval)
- **Hernieuwde subscriptie**: Start als nieuwe verbinding (afhankelijk van reset instellingen)

## 🎯 Gedetailleerde Controle Opties (RxJS 7+)

In RxJS 7 en later kunt u het gedrag fijn afstemmen door opties door te geven aan `share()`.

```typescript
import { interval, share, take, tap } from 'rxjs';

const source$ = interval(1000).pipe(
  take(6),
  tap((value) => console.log(`Bron: ${value}`)),
  share({
    resetOnError: true,       // Reset bij fout
    resetOnComplete: true,     // Reset bij voltooiing
    resetOnRefCountZero: true, // Reset wanneer subscribers 0 worden
  })
);
```

### Optie Details

| Optie | Standaard | Beschrijving |
|-------|----------|--------------|
| `resetOnError` | `true` | Reset interne status bij fout |
| `resetOnComplete` | `true` | Reset interne status bij stream voltooiing |
| `resetOnRefCountZero` | `true` | Verbreek verbinding wanneer subscribers 0 worden |
| `connector` | `() => new Subject()` | Specificeer aangepaste Subject |

### Geavanceerde Controle met connector Optie

Door de `connector` optie te gebruiken, kunt u gedrag equivalent aan `shareReplay` realiseren.

```typescript
import { interval, ReplaySubject } from 'rxjs';
import { take, share, tap } from 'rxjs';

// Buffer laatste 1 item met ReplaySubject
const source$ = interval(1000).pipe(
  take(5),
  tap(value => console.log(`Bron: ${value}`)),
  share({
    connector: () => new ReplaySubject(1),
    resetOnError: false,
    resetOnComplete: false,
    resetOnRefCountZero: false
  })
);

// Eerste subscriber
source$.subscribe(value => console.log(`Observer 1: ${value}`));

// Subscribe na 2,5 seconden (ontvang afgelopen 1 item)
setTimeout(() => {
  source$.subscribe(value => console.log(`Observer 2: ${value}`));
}, 2500);
```

**Uitvoeringsresultaat**:
```
Bron: 0
Observer 1: 0
Bron: 1
Observer 1: 1
Observer 2: 1  // ← Ontvangt vorige waarde zelfs bij tussentijdse deelname
Bron: 2
Observer 1: 2
Observer 2: 2
...
```

> [!TIP]
> Deze methode kan worden gebruikt als alternatief voor `shareReplay(1)`. Door `resetOnRefCountZero: false` in te stellen, behoudt u de verbinding zelfs wanneer de reference count 0 wordt, waarmee u het "permanente cache" probleem van `shareReplay` vermijdt.

## 📊 Vergelijking met en zonder share()

### ❌ Zonder share() (Cold Observable)

```typescript
import { interval, take, tap } from 'rxjs';

const source$ = interval(1000).pipe(
  take(3),
  tap(value => console.log(`Bron: ${value}`))
);

// Subscriber 1
source$.subscribe(value => console.log(`Observer 1: ${value}`));

// Subscriber 2
setTimeout(() => {
  source$.subscribe(value => console.log(`Observer 2: ${value}`));
}, 1500);
```

**Uitvoeringsresultaat**:
```
Bron: 0
Observer 1: 0
Bron: 1
Observer 1: 1
Bron: 0    // ← Nieuwe stream wordt gestart
Observer 2: 0
Bron: 2
Observer 1: 2
Bron: 1
Observer 2: 1
Bron: 2
Observer 2: 2
```

Elke subscriber heeft een onafhankelijke stream en bronverwerking wordt dubbel uitgevoerd.

### ✅ Met share() (Hot Observable)

```typescript
import { interval, share, take, tap } from 'rxjs';

const source$ = interval(1000).pipe(
  take(3),
  tap(value => console.log(`Bron: ${value}`)),
  share()
);

// Subscriber 1
source$.subscribe(value => console.log(`Observer 1: ${value}`));

// Subscriber 2
setTimeout(() => {
  source$.subscribe(value => console.log(`Observer 2: ${value}`));
}, 1500);
```

**Uitvoeringsresultaat**:
```
Bron: 0
Observer 1: 0
Bron: 1
Observer 1: 1
Observer 2: 1  // ← Deelt dezelfde stream
Bron: 2
Observer 1: 2
Observer 2: 2
```

## 💼 Praktische Use Cases

### Voorkomen van Dubbele API Verzoeken

```typescript
import { ajax } from 'rxjs/ajax';
import { share, tap } from 'rxjs';

// Observable om gebruikersinformatie op te halen
const getUser$ = ajax.getJSON('https://jsonplaceholder.typicode.com/users/1').pipe(
  tap(() => console.log('API verzoek uitvoeren')),
  share() // Voorkom dubbele verzoeken in meerdere componenten
);

// Component 1
getUser$.subscribe(user => console.log('Component 1:', user));

// Component 2 (verzoek bijna gelijktijdig)
getUser$.subscribe(user => console.log('Component 2:', user));

// Resultaat: API verzoek wordt slechts één keer uitgevoerd
```

### Periodieke Gegevensophaling Delen

```typescript
import { timer, share, switchMap, tap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

// TODO lijst ophalen elke 5 seconden (API verzoek wordt gedeeld)
const sharedTodos$ = timer(0, 5000).pipe(
  tap(() => console.log('API verzoek uitvoeren')),
  switchMap(() => ajax.getJSON('https://jsonplaceholder.typicode.com/todos?_limit=3')),
  share() // Deel API verzoek voor meerdere subscribers
);

// Gebruik dezelfde data stream in meerdere componenten
sharedTodos$.subscribe(todos => console.log('Component A:', todos));
sharedTodos$.subscribe(todos => console.log('Component B:', todos));

// Resultaat: API verzoek wordt elke 5 seconden slechts één keer uitgevoerd en beide componenten ontvangen dezelfde gegevens
```

## ⚠️ Aandachtspunten

1. **Let op timing**: Tussentijds deelnemende subscribers kunnen geen vorige waarden ontvangen
2. **Fout propagatie**: Wanneer een fout optreedt, beïnvloedt dit alle subscribers
3. **Geheugenbeheer**: Als u subscripties niet goed afmeldt, kan dit leiden tot geheugenlekkages

## 🔄 Gerelateerde Operators

- **[shareReplay()](/nl/guide/operators/multicasting/shareReplay)** - Buffert vorige waarden en biedt deze ook aan volgende subscribers
- **[Subject](/nl/guide/subjects/what-is-subject)** - Klasse die de basis vormt voor multicasting

> [!WARNING]
> **Verouderde Operators**: Oude multicasting API's zoals `publish()`, `multicast()`, `refCount()` zijn verouderd in RxJS v7 en verwijderd in v8. Gebruik in plaats daarvan `share()` of `connectable()`/`connect()`.

## Samenvatting

De `share()` operator,
- Deelt dezelfde Observable met meerdere subscribers
- Voorkomt dubbele uitvoering van API verzoeken en zware verwerking
- Basis van eenvoudig te gebruiken multicasting
- Gedetailleerde controle opties beschikbaar in RxJS 7+

Wanneer meerdere componenten dezelfde gegevensbron nodig hebben, kunt u de prestaties aanzienlijk verbeteren door `share()` te gebruiken.
