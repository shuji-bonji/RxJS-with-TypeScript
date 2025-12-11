---
description: Subject is een speciale klasse in RxJS die zowel de eigenschappen van een Observable als een Observer heeft. Het kan tegelijkertijd data uitsturen en erop abonneren, en via multicasting dezelfde waarden aan meerdere abonnees versturen. Met type-parameters in TypeScript behoud je type-veiligheid terwijl je praktische patronen zoals event busses en state management implementeert.
---

# Wat is een Subject

[📘 RxJS Officieel: Subject](https://rxjs.dev/api/index/class/Subject)

Een Subject is een speciaal type Observable in RxJS. Terwijl een gewone Observable een unidirectionele datastroom biedt, is een Subject een hybride entiteit die zowel de eigenschappen van een "Observable" als een "Observer" heeft.

Een Subject heeft de volgende kenmerken:

- Kan data uitsturen (Observable-functionaliteit)
- Kan abonneren op data (Observer-functionaliteit)
- Kan dezelfde waarden aan meerdere abonnees leveren (multicasting)
- Ontvangt alleen waarden die na het abonnement plaatsvinden (Hot Observable-achtige eigenschap)


## Basis gebruik van Subject

```ts
import { Subject } from 'rxjs';

// Maak een Subject
const subject = new Subject<number>();

// Abonneer als Observer
subject.subscribe(value => console.log('Observer A:', value));
subject.subscribe(value => console.log('Observer B:', value));

// Stuur waarden uit als Observable
subject.next(1); // Stuurt waarde naar beide abonnees
subject.next(2); // Stuurt waarde naar beide abonnees

// Voeg een nieuwe abonnee toe (late abonnee)
subject.subscribe(value => console.log('Observer C:', value));

subject.next(3); // Stuurt waarde naar alle abonnees

// Meld voltooiing aan
subject.complete();
```

#### Uitvoerresultaat
```
Observer A: 1
Observer B: 1
Observer A: 2
Observer B: 2
Observer A: 3
Observer B: 3
Observer C: 3
```

### Verschil met gewone Observable

Subject is een **Hot Observable** en verschilt van gewone Cold Observables op de volgende punten:

- Data wordt uitgezonden ongeacht of er abonnees zijn
- Meerdere abonnees kunnen dezelfde waarden delen (multicasting)
- Waarden kunnen extern worden uitgezonden met `.next()`
- Er worden geen eerdere waarden bewaard; alleen waarden na het abonnement worden ontvangen


## Subject en Multicasting

Een van de belangrijke functies van Subject is "multicasting".
Dit is de mogelijkheid om één databron efficiënt aan meerdere abonnees te distribueren.

```ts
import { Subject, interval } from 'rxjs';
import { take } from 'rxjs';

// Databron
const source$ = interval(1000).pipe(take(3));

// Subject voor multicasting
const subject = new Subject<number>();

// Verbind bron met Subject
source$.subscribe(subject); // Subject fungeert als abonnee

// Meerdere observers abonneren op Subject
subject.subscribe(value => console.log('Observer 1:', value));
subject.subscribe(value => console.log('Observer 2:', value));
```

#### Uitvoerresultaat
```
Observer 1: 0
Observer 2: 0
Observer 1: 1
Observer 2: 1
Observer 1: 2
Observer 2: 2
```

Dit patroon wordt ook wel single-source-multicast genoemd en wordt gebruikt om één databron efficiënt aan meerdere abonnees te distribueren.


## Twee manieren om Subject te gebruiken

Subject heeft hoofdzakelijk twee gebruikspatronen. Elk heeft verschillende toepassingen en gedragingen.

### 1. Patroon waarbij je zelf `.next()` aanroept

Subject wordt gebruikt als **primaire bron van data (Observable)**.
Dit patroon is geschikt voor "expliciete waarde-uitzendings"-scenario's zoals event-notificaties en status-updates.

```ts
const subject = new Subject<string>();

subject.subscribe(val => console.log('Observer A:', val));
subject.next('Hello');
subject.next('World');

// Uitvoer:
// Observer A: Hello
// Observer A: World
```

---

### 2. Patroon waarbij Observable wordt doorgegeven (multicast)

Subject fungeert als **Observer die waarden ontvangt van Observable en deze doorgeeft**.
Deze manier van gebruiken is handig om **Cold Observable naar Hot te converteren en te multicasten**.

```ts
const source$ = interval(1000).pipe(take(3));
const subject = new Subject<number>();

// Observable → Subject (relay)
source$.subscribe(subject);

// Subject → distributie naar meerdere abonnees
subject.subscribe(val => console.log('Observer 1:', val));
subject.subscribe(val => console.log('Observer 2:', val));

// Uitvoer:
// Observer 1: 0
// Observer 2: 0
// Observer 1: 1
// Observer 2: 1
// Observer 1: 2
// Observer 2: 2
```



> [!TIP]
> Als je zelf `.next()` aanroept, stel je het voor als "iemand die zelf spreekt". Bij het ontvangen en doorgeven van Observable is het als "iemand die andermans woorden via een microfoon versterkt". Dit maakt het gemakkelijker te begrijpen.


## Praktische use cases van Subject

Subject is bijzonder nuttig in de volgende scenario's:

1. **State management** - Status van applicatie delen en bijwerken
2. **Event bus** - Communicatie tussen componenten
3. **HTTP response delen** - Resultaten van dezelfde API-call delen tussen meerdere componenten
4. **Gecentraliseerd UI-event beheer** - Verschillende UI-operaties op één plek verwerken

#### Voorbeeld: Implementatie van event bus
```ts
import { Subject } from 'rxjs';
import { filter } from 'rxjs';

interface AppEvent {
  type: string;
  payload: any;
}

// Event bus voor hele applicatie
const eventBus = new Subject<AppEvent>();

// Abonneer op specifiek event type
eventBus.pipe(
  filter(event => event.type === 'USER_LOGGED_IN')
).subscribe(event => {
  console.log('Gebruiker ingelogd:', event.payload);
});

// Abonneer op ander event type
eventBus.pipe(
  filter(event => event.type === 'DATA_UPDATED')
).subscribe(event => {
  console.log('Data bijgewerkt:', event.payload);
});

// Stuur events uit
eventBus.next({ type: 'USER_LOGGED_IN', payload: { userId: '123', username: 'test_user' } });
eventBus.next({ type: 'DATA_UPDATED', payload: { items: [1, 2, 3] } });
```

#### Uitvoerresultaat
```
Gebruiker ingelogd: {userId: '123', username: 'test_user'}
Data bijgewerkt: {items: Array(3)}
```

## Samenvatting

Subject is een belangrijk onderdeel van het RxJS-ecosysteem dat de volgende rollen vervult:

- Heeft eigenschappen van zowel Observer (observeerder) als Observable (geobserveerde)
- Biedt middelen om Cold Observable naar Hot te converteren
- Distribueert efficiënt dezelfde datastroom naar meerdere abonnees
- Vergemakkelijkt communicatie tussen componenten en services
- Biedt basis voor state management en event handling

## 🔗 Gerelateerde secties

- **[Veelvoorkomende fouten en oplossingen](/nl/guide/anti-patterns/common-mistakes#1-subject-の外部公開)** - Best practices om misbruik van Subject te vermijden
- **[Soorten Subjects](./types-of-subject)** - BehaviorSubject, ReplaySubject, AsyncSubject, etc.
