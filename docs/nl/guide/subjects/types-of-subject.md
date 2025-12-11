---
description: "Uitleg over de 4 soorten Subjects (Subject, BehaviorSubject, ReplaySubject, AsyncSubject) en hun kenmerken en toepassingsscenario's. Leer het verschil in gebruik op basis van de aanwezigheid van initiële waarde, aantal herhalingen van waarden, ophalen van waarden na voltooiing, etc. met TypeScript-codevoorbeelden."
---

# Soorten Subjects

Naast de basis `Subject` biedt RxJS meerdere afgeleide klassen die gespecialiseerd zijn in specifieke use cases. Elk heeft verschillende gedragskenmerken, en door ze in de juiste situatie toe te passen wordt effectiever reactief programmeren mogelijk.

Hier leggen we de 4 belangrijkste soorten Subjects, hun kenmerken en toepassingsscenario's in detail uit.

## De 4 basis soorten Subjects

| Soort | Kenmerken | Belangrijkste Use Cases |
|------|------|----------------|
| [`Subject`](#subject) | Meest eenvoudige Subject<br>Ontvangt alleen waarden na abonnement | Event notificatie, multicasting |
| [`BehaviorSubject`](#behaviorsubject) | Behoudt laatste waarde en levert deze direct bij nieuw abonnement | State management, huidige waarde van UI-componenten |
| [`ReplaySubject`](#replaysubject) | Speelt opgegeven aantal eerdere waarden opnieuw af voor nieuwe abonnees | Operationele geschiedenis, recente update-informatie |
| [`AsyncSubject`](#asyncsubject) | Zendt alleen laatste waarde bij voltooiing uit | HTTP/API request resultaten |

## Standaard `Subject` {#subject}

[📘 RxJS Officieel: Subject](https://rxjs.dev/api/index/class/Subject)

Het meest eenvoudige type Subject, ontvangt alleen waarden die na het abonnement plaatsvinden.

```ts
import { Subject } from 'rxjs';

const subject = new Subject<number>();

// Geen initiële waarde, ontvangt niets bij abonnement
subject.subscribe(value => console.log('Observer 1:', value));

subject.next(1);
subject.next(2);

// Tweede abonnement (ontvangt alleen waarden na abonnement)
subject.subscribe(value => console.log('Observer 2:', value));

subject.next(3);
subject.complete();
```

#### Uitvoerresultaat
```
Observer 1: 1
Observer 1: 2
Observer 1: 3
Observer 2: 3
```

## BehaviorSubject  {#behaviorsubject}

[📘 RxJS Officieel: BehaviorSubject](https://rxjs.dev/api/index/class/BehaviorSubject)

Vereist een initiële waarde en behoudt altijd de laatste waarde.
Nieuwe abonnees ontvangen onmiddellijk de laatste waarde bij abonnement.

```ts
import { BehaviorSubject } from 'rxjs';

// Maak met initiële waarde 0
const behaviorSubject = new BehaviorSubject<number>(0);

// Ontvangt direct initiële waarde
behaviorSubject.subscribe(value => console.log('Observer 1:', value));

behaviorSubject.next(1);
behaviorSubject.next(2);

// Tweede abonnement (ontvangt direct laatste waarde 2)
behaviorSubject.subscribe(value => console.log('Observer 2:', value));

behaviorSubject.next(3);
behaviorSubject.complete();
```

#### Uitvoerresultaat
```
Observer 1: 0
Observer 1: 1
Observer 1: 2
Observer 2: 2
Observer 1: 3
Observer 2: 3
```

### Toepassingsvoorbeeld van BehaviorSubject

#### Beheer van gebruikersauthenticatiestatus

```ts
import { BehaviorSubject } from 'rxjs';

interface User {
  id: string;
  name: string;
}

// Initiële waarde is null (niet ingelogd)
const currentUser$ = new BehaviorSubject<User | null>(null);

// Monitor login status in component etc.
currentUser$.subscribe(user => {
  if (user) {
    console.log(`Ingelogd: ${user.name}`);
  } else {
    console.log('Niet ingelogd');
  }
});

// Login verwerking
function login(user: User) {
  currentUser$.next(user);
}

// Logout verwerking
function logout() {
  currentUser$.next(null);
}

// Gebruiksvoorbeeld
console.log('Applicatie opgestart');
// → Niet ingelogd

login({ id: 'user123', name: 'Yamada Taro' });
// → Ingelogd: Yamada Taro

logout();
// → Niet ingelogd
```

#### Uitvoerresultaat
```sh
Niet ingelogd
Applicatie opgestart
Ingelogd: Yamada Taro
Niet ingelogd
```

## `ReplaySubject` {#replaysubject}
[📘 RxJS Officieel: ReplaySubject](https://rxjs.dev/api/index/class/ReplaySubject)

Onthoudt opgegeven aantal eerdere waarden en stuurt deze opnieuw naar nieuwe abonnees.
Buffergrootte en tijdvenster kunnen worden ingesteld.

```ts
import { ReplaySubject } from 'rxjs';

// Buffer laatste 3 waarden
const replaySubject = new ReplaySubject<number>(3);

replaySubject.next(1);
replaySubject.next(2);
replaySubject.next(3);
replaySubject.next(4);

// Start abonnement (ontvangt laatste 3 waarden 2,3,4)
replaySubject.subscribe(value => console.log('Observer 1:', value));

replaySubject.next(5);

// Tweede abonnement (ontvangt laatste 3 waarden 3,4,5)
replaySubject.subscribe(value => console.log('Observer 2:', value));

replaySubject.complete();
```

#### Uitvoerresultaat
```
Observer 1: 2
Observer 1: 3
Observer 1: 4
Observer 1: 5
Observer 2: 3
Observer 2: 4
Observer 2: 5
```

### ReplaySubject met tijdvenster

Bufferen op basis van tijd is ook mogelijk.

```ts
import { ReplaySubject } from 'rxjs';

// Maximaal 5 waarden bufferen, en waarden binnen 500ms
const timeWindowSubject = new ReplaySubject<number>(5, 500);

timeWindowSubject.next(1);

setTimeout(() => {
  timeWindowSubject.next(2);

  // Abonneer na 1000ms (1 wordt niet ontvangen omdat 500ms tijdvenster is overschreden)
  setTimeout(() => {
    timeWindowSubject.subscribe(value => console.log('Ontvangen:', value));
  }, 1000);
}, 100);
```

#### Uitvoerresultaat
```
Ontvangen: 2
```

### Toepassingsvoorbeeld van ReplaySubject

#### Beheer van recente zoekgeschiedenis

```ts
import { ReplaySubject } from 'rxjs';

// Behoud laatste 5 zoekqueries
const searchHistory$ = new ReplaySubject<string>(5);

// Zoekuitvoeringsfunctie
function search(query: string) {
  console.log(`Zoekopdracht uitgevoerd: ${query}`);
  searchHistory$.next(query);
  // Werkelijke zoekverwerking...
}

// Component die zoekgeschiedenis toont
function showSearchHistory() {
  console.log('--- Zoekgeschiedenis ---');
  searchHistory$.subscribe(query => {
    console.log(query);
  });
}

// Gebruiksvoorbeeld
search('TypeScript');
search('RxJS');
search('Angular');
search('React');

showSearchHistory();
// Toont laatste 5 (in dit geval 4) zoekgeschiedenissen
```

#### Uitvoerresultaat
```sh
Zoekopdracht uitgevoerd: TypeScript
Zoekopdracht uitgevoerd: RxJS
Zoekopdracht uitgevoerd: Angular
Zoekopdracht uitgevoerd: React
--- Zoekgeschiedenis ---
TypeScript
RxJS
Angular
React
```

## `AsyncSubject` {#asyncsubject}
[📘 RxJS Officieel: AsyncSubject](https://rxjs.dev/api/index/class/AsyncSubject)

Subject dat alleen de laatste waarde bij voltooiing uitzendt. Waarden vóór voltooiing worden niet uitgezonden.

```ts
import { AsyncSubject } from 'rxjs';

const asyncSubject = new AsyncSubject<number>();

asyncSubject.subscribe(value => console.log('Observer 1:', value));

asyncSubject.next(1);
asyncSubject.next(2);
asyncSubject.next(3);

// Ongeacht timing van abonnement, ontvangt alleen laatste waarde
asyncSubject.subscribe(value => console.log('Observer 2:', value));

asyncSubject.next(4);
asyncSubject.complete(); // Bij voltooiing wordt laatste waarde (4) uitgezonden
```

#### Uitvoerresultaat
```
Observer 1: 4
Observer 2: 4
```

### Toepassingsvoorbeeld van AsyncSubject

#### Delen van API request resultaat

```ts
import { AsyncSubject } from 'rxjs';

interface ApiResponse {
  data: any;
  status: number;
}

function fetchData(url: string) {
  const subject = new AsyncSubject<ApiResponse>();

  // Simuleer API request
  console.log(`API request: ${url}`);
  setTimeout(() => {
    const response = {
      data: { id: 1, name: 'Voorbeelddata' },
      status: 200
    };

    subject.next(response);
    subject.complete();
  }, 1000);

  return subject;
}

// Gebruiksvoorbeeld
const data$ = fetchData('/api/users/1');

// Meerdere componenten kunnen hetzelfde request resultaat delen
data$.subscribe(response => {
  console.log('Component 1:', response.data);
});

setTimeout(() => {
  data$.subscribe(response => {
    console.log('Component 2:', response.data);
  });
}, 1500); // Kan waarde ontvangen zelfs na voltooiing
```

#### Uitvoerresultaat
```sh
API request: /api/users/1
Component 1: {id: 1, name: 'Voorbeelddata'}
Component 2: {id: 1, name: 'Voorbeelddata'}
```

## Vergelijking en Selectiegids van elk Subject

We vatten nuttige punten samen bij het kiezen van elk Subject-type.

### Hoe Subject te kiezen

|type|Selectiecriteria|
|---|---|
|`Subject`|Gebruik voor eenvoudige event notificatie of multicast distributie|
|`BehaviorSubject`|<li>Wanneer initiële waarde altijd nodig is </li><li>Data die huidige status vertegenwoordigt (gebruikersstatus, instellingen, vlaggen etc.) </li><li>Huidige waarde van UI-componenten</li>|
|`ReplaySubject`|<li>Wanneer recente operationele geschiedenis moet worden behouden </li><li>Wanneer eerdere data aan later toegevoegde abonnees moet worden geleverd </li><li>Gebufferde datastromen</li>|
|`AsyncSubject`|<li>Wanneer alleen eindresultaat belangrijk is (zoals API responses) </li><li>Wanneer tussentijdse voortgang niet nodig is en alleen waarde bij voltooiing moet worden gedeeld</li>|

### Beslissingsstroomdiagram voor selectie

1. Alleen laatste waarde bij voltooiing nodig ⇨ `AsyncSubject`
2. Laatste N waarden nodig ⇨ `ReplaySubject`
3. Huidige status/waarde altijd nodig ⇨ `BehaviorSubject`
4. Anders (pure event notificatie etc.) ⇨ `Subject`

## Toepassingspatronen in applicatie-ontwerp

### Voorbeeld van communicatie tussen modules

```ts
// Status management service voor hele applicatie
class AppStateService {
  // Huidige gebruiker (BehaviorSubject omdat initiële waarde verplicht)
  private userSubject = new BehaviorSubject<User | null>(null);
  // Publiceer als read-only Observable
  readonly user$ = this.userSubject.asObservable();

  // Notificatie (Subject omdat eenvoudige event notificatie)
  private notificationSubject = new Subject<Notification>();
  readonly notifications$ = this.notificationSubject.asObservable();

  // Recente zoekopdrachten (ReplaySubject omdat geschiedenis nodig)
  private searchHistorySubject = new ReplaySubject<string>(10);
  readonly searchHistory$ = this.searchHistorySubject.asObservable();

  // API call resultaat cache (AsyncSubject omdat alleen eindresultaat nodig)
  private readonly apiCaches = new Map<string, AsyncSubject<any>>();

  // Voorbeeldmethoden
  setUser(user: User | null) {
    this.userSubject.next(user);
  }

  notify(notification: Notification) {
    this.notificationSubject.next(notification);
  }

  addSearch(query: string) {
    this.searchHistorySubject.next(query);
  }

  // Cache van API resultaat
  fetchData(url: string): Observable<any> {
    if (!this.apiCaches.has(url)) {
      const subject = new AsyncSubject<any>();
      this.apiCaches.set(url, subject);

      // Werkelijke API call
      fetch(url)
        .then(res => res.json())
        .then(data => {
          subject.next(data);
          subject.complete();
        })
        .catch(err => {
          subject.error(err);
        });
    }

    return this.apiCaches.get(url)!.asObservable();
  }
}
```

## Samenvatting

Subject in RxJS is een krachtig hulpmiddel dat verschillende use cases kan dekken. Door de kenmerken van elk type te begrijpen en ze op de juiste manier toe te passen, kun je efficiënte en onderhoudbare reactieve applicaties bouwen.

- `Subject`: Meest eenvoudig en biedt basis multicast functionaliteit
- `BehaviorSubject`: Behoudt altijd huidige status en levert deze direct aan nieuwe abonnees
- `ReplaySubject`: Behoudt geschiedenis van recente waarden en levert deze ook aan later toegevoegde abonnees
- `AsyncSubject`: Zendt alleen laatste waarde bij voltooiing uit

Het kiezen van het juiste `Subject` voor state management, event notificatie, data delen etc. in alle situaties is de sleutel tot efficiënt reactief programmeren.
