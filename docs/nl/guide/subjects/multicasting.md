---
description: Gedetailleerde uitleg over het mechanisme van multicasting in RxJS. We introduceren praktische ontwerppatronen met TypeScript-codevoorbeelden, waaronder basispatronen met Subject, het gebruik van share en shareReplay operators, het vermijden van dubbele API-requests, cache-strategieën, het delen van applicatiestatus en meer.
---

# Het mechanisme van Multicasting

Multicasting is een techniek om een datastroom van één Observable efficiënt te distribueren naar meerdere abonnees (Observers).
In RxJS kan dit worden gerealiseerd met Subjects en operators.

## Wat is Multicasting

Een gewone Observable (Cold Observable) creëert een nieuwe datastroom elke keer dat er een abonnement wordt genomen. Dit betekent dat bij meerdere abonnees dezelfde verwerking meerdere keren wordt uitgevoerd.

Door multicasting te gebruiken, kan de databron slechts één keer worden uitgevoerd en het resultaat naar meerdere abonnees worden gedistribueerd. Dit is vooral belangrijk in de volgende gevallen:

- Je wilt HTTP/API-requests niet dubbel aanroepen
- Je wilt dure operaties (berekeningen of side effects) slechts één keer uitvoeren
- De applicatiestatus delen tussen meerdere componenten

## Basispatroon van Multicasting

### Basis multicast met Subject

```ts
import { Observable, Subject } from 'rxjs';
import { tap } from 'rxjs';

// Databron (Cold Observable)
function createDataSource(): Observable<number> {
  return new Observable<number>(observer => {
    console.log('Databron: verbonden');
    // Datageneratie logica (uitgaande van dure operatie)
    const id = setInterval(() => {
      const value = Math.round(Math.random() * 100);
      console.log(`Databron: waarde gegenereerd -> ${value}`);
      observer.next(value);
    }, 1000);

    // Cleanup functie
    return () => {
      console.log('Databron: verbinding verbroken');
      clearInterval(id);
    };
  });
}

// Multicast implementatie
function multicast() {
  // Originele databron
  const source$ = createDataSource().pipe(
    tap(value => console.log(`Bronverwerking: ${value}`))
  );

  // Subject voor multicasting
  const subject = new Subject<number>();

  // Verbind bron met Subject
  const subscription = source$.subscribe(subject);

  // Meerdere abonnees abonneren op Subject
  console.log('Observer 1 start abonnement');
  const subscription1 = subject.subscribe(value => console.log(`Observer 1: ${value}`));

  // Voeg na 3 seconden andere abonnee toe
  setTimeout(() => {
    console.log('Observer 2 start abonnement');
    const subscription2 = subject.subscribe(value => console.log(`Observer 2: ${value}`));

    // Beëindig na 5 seconden alle abonnementen
    setTimeout(() => {
      console.log('Beëindig alle abonnementen');
      subscription.unsubscribe();
      subscription1.unsubscribe();
      subscription2.unsubscribe();
    }, 5000);
  }, 3000);
}

// Uitvoeren
multicast();
```

#### Uitvoerresultaat
```
Databron: verbonden
Observer 1 start abonnement
Databron: waarde gegenereerd -> 71
Bronverwerking: 71
Observer 1: 71
Databron: waarde gegenereerd -> 79
Bronverwerking: 79
Observer 1: 79
Databron: waarde gegenereerd -> 63
Bronverwerking: 63
Observer 1: 63
Observer 2 start abonnement
Databron: waarde gegenereerd -> 49
Bronverwerking: 49
Observer 1: 49
Observer 2: 49
Databron: waarde gegenereerd -> 94
Bronverwerking: 94
Observer 1: 94
Observer 2: 94
Databron: waarde gegenereerd -> 89
Bronverwerking: 89
Observer 1: 89
Observer 2: 89
Databron: waarde gegenereerd -> 10
Bronverwerking: 10
Observer 1: 10
Observer 2: 10
Databron: waarde gegenereerd -> 68
Bronverwerking: 68
Observer 1: 68
Observer 2: 68
Beëindig alle abonnementen
Databron: verbinding verbroken
```

## Multicast operators

RxJS biedt specifieke operators voor het implementeren van multicasting.

### `share()` operator
[📘 RxJS Officieel: share()](https://rxjs.dev/api/index/function/share)

De eenvoudigste operator om multicast te implementeren.
Intern combineert het `multicast()` en `refCount()`.

```ts
import { interval } from 'rxjs';
import { take, share, tap } from 'rxjs';

// Observable die telt met interval
const source$ = interval(1000).pipe(
  take(5),
  tap(value => console.log(`Bron: ${value}`)),
  share() // Schakel multicasting in
);

// Eerste abonnee
console.log('Observer 1 start abonnement');
const subscription1 = source$.subscribe(value => console.log(`Observer 1: ${value}`));

// Voeg na 2.5 seconden tweede abonnee toe
setTimeout(() => {
  console.log('Observer 2 start abonnement');
  const subscription2 = source$.subscribe(value => console.log(`Observer 2: ${value}`));

  // Hef na 5 seconden abonnee 1 op
  setTimeout(() => {
    console.log('Observer 1 abonnement opgeheven');
    subscription1.unsubscribe();
  }, 2500);
}, 2500);
```

#### Uitvoerresultaat
```
Observer 1 start abonnement
Bron: 0
Observer 1: 0
Observer 2 start abonnement
Bron: 1
Observer 1: 1
Observer 2: 1
Bron: 2
Observer 1: 2
Observer 2: 2
Bron: 3
Observer 1: 3
Observer 2: 3
Observer 1 abonnement opgeheven
Bron: 4
Observer 2: 4
```

### Gedetailleerde controle van `share()`

Vanaf RxJS 7 kan het gedrag duidelijker worden gecontroleerd door opties aan `share()` mee te geven in plaats van `refCount()`.

```ts
import { interval } from 'rxjs';
import { take, share, tap } from 'rxjs';

const source$ = interval(1000).pipe(
  take(6),
  tap((value) => console.log(`Bron: ${value}`)),
  share({
    resetOnError: true,
    resetOnComplete: true,
    resetOnRefCountZero: true,
  })
);

// Eerste abonnee
console.log('Observer 1 start abonnement');
const subscription1 = source$.subscribe((value) =>
  console.log(`Observer 1: ${value}`)
);

// Voeg na 2.5 seconden tweede abonnee toe
setTimeout(() => {
  console.log('Observer 2 start abonnement');
  const subscription2 = source$.subscribe((value) =>
    console.log(`Observer 2: ${value}`)
  );

  setTimeout(() => {
    console.log('Observer 1 abonnement opgeheven');
    subscription1.unsubscribe();
  }, 1500);
}, 2500);
```

#### Uitvoerresultaat
```
Observer 1 start abonnement
Bron: 0
Observer 1: 0
Bron: 1
Observer 1: 1
Observer 2 start abonnement
Bron: 2
Observer 1: 2
Observer 2: 2
Bron: 3
Observer 1: 3
Observer 2: 3
Observer 1 abonnement opgeheven
Bron: 4
Observer 2: 4
Bron: 5
Observer 2: 5
```

Met deze methode kan het gedrag bij beëindiging van de stream of wanneer abonnees nul worden, duidelijk worden gecontroleerd.

### `shareReplay()` operator

[📘 RxJS Officieel: shareReplay()](https://rxjs.dev/api/index/function/shareReplay)

Vergelijkbaar met `share()`, maar onthoudt een opgegeven aantal eerdere waarden en biedt deze ook aan later toegevoegde abonnees.

```ts
import { interval } from 'rxjs';
import { take, shareReplay, tap } from 'rxjs';

// Gebruik shareReplay (buffergrootte 2)
const source$ = interval(1000).pipe(
  take(5),
  tap(value => console.log(`Bron: ${value}`)),
  shareReplay(2) // Buffer de laatste 2 waarden
);

// Eerste abonnee
console.log('Observer 1 start abonnement');
source$.subscribe(value => console.log(`Observer 1: ${value}`));

// Voeg na 3.5 seconden tweede abonnee toe
setTimeout(() => {
  console.log('Observer 2 start abonnement - ontvangt laatste 2 waarden');
  source$.subscribe(value => console.log(`Observer 2: ${value}`));
}, 3500);
```

#### Uitvoerresultaat
```
Observer 1 start abonnement
Bron: 0
Observer 1: 0
Bron: 1
Observer 1: 1
Observer 2 start abonnement - ontvangt laatste 2 waarden
Observer 2: 0
Observer 2: 1
Bron: 2
Observer 1: 2
Observer 2: 2
Bron: 3
Observer 1: 3
Observer 2: 3
Bron: 4
Observer 1: 4
Observer 2: 4
```

## Timing en Lifecycle in Multicasting

Het is belangrijk om de lifecycle van multicast streams te begrijpen. Vooral bij gebruik van de `share()` operator moet je letten op het volgende gedrag:

1. Eerste abonnee: `share()` start de verbinding met de bron Observable wanneer het eerste abonnement plaatsvindt.
2. Alle abonnees worden opgeheven: Bij de instelling `share({ resetOnRefCountZero: true })` wordt de verbinding met de bron verbroken wanneer abonnees nul worden.
3. Complete of Error: Standaard reset `share()` de interne staat wanneer complete of error optreedt (wanneer resetOnComplete/resetOnError true is).
4. Re-subscribe: Wanneer na reset van de stream opnieuw wordt geabonneerd, wordt deze als nieuwe Observable herbouwd.

Op deze manier wordt de timing van start, stop en regeneratie van streams gecontroleerd door de opties van `share()`, afhankelijk van het aantal abonnees en de complete status.

## Praktische Use Cases

### API-requests delen

Voorbeeld om dubbele requests naar hetzelfde API-endpoint te vermijden.

```ts
import { Observable, of, throwError } from 'rxjs';
import { ajax } from 'rxjs/ajax';
import { map, catchError, shareReplay, tap } from 'rxjs';

// Simulatie van API-service
class UserService {
  private cache = new Map<string, Observable<any>>();

  getUser(id: string): Observable<any> {
    // Als in cache, retourneer die
    if (this.cache.has(id)) {
      console.log(`Gebruiker ID ${id} ophalen uit cache`);
      return this.cache.get(id)!;
    }

    // Maak nieuwe request
    console.log(`Gebruiker ID ${id} ophalen via API`);
    const request$ = ajax.getJSON(`https://jsonplaceholder.typicode.com/users/${id}`).pipe(
      tap(response => console.log('API Response:', response)),
      catchError(error => {
        console.error('API Error:', error);
        // Verwijder uit cache
        this.cache.delete(id);
        return throwError(() => new Error('Ophalen gebruiker mislukt'));
      }),
      // Deel met shareReplay (cache waarde ook na complete)
      shareReplay(1)
    );

    // Opslaan in cache
    this.cache.set(id, request$);
    return request$;
  }
}

// Gebruiksvoorbeeld
const userService = new UserService();

// Meerdere componenten vragen dezelfde gebruikersdata
console.log('Component 1: vraagt gebruikersdata');
userService.getUser('1').subscribe(user => {
  console.log('Component 1: ontvangt gebruikersdata', user);
});

// Iets later vraagt ander component dezelfde data
setTimeout(() => {
  console.log('Component 2: vraagt dezelfde gebruikersdata');
  userService.getUser('1').subscribe(user => {
    console.log('Component 2: ontvangt gebruikersdata', user);
  });
}, 1000);

// Vraag andere gebruiker
setTimeout(() => {
  console.log('Component 3: vraagt andere gebruikersdata');
  userService.getUser('2').subscribe(user => {
    console.log('Component 3: ontvangt gebruikersdata', user);
  });
}, 2000);
```

#### Uitvoerresultaat
```
Component 1: vraagt gebruikersdata
Gebruiker ID 1 ophalen via API
API Response: {id: 1, name: 'Leanne Graham', username: 'Bret', email: 'Sincere@april.biz', address: {…}, …}
Component 1: ontvangt gebruikersdata {id: 1, name: 'Leanne Graham', username: 'Bret', email: 'Sincere@april.biz', address: {…}, …}
Component 2: vraagt dezelfde gebruikersdata
Gebruiker ID 1 ophalen uit cache
Component 2: ontvangt gebruikersdata {id: 1, name: 'Leanne Graham', username: 'Bret', email: 'Sincere@april.biz', address: {…}, …}
Component 3: vraagt andere gebruikersdata
Gebruiker ID 2 ophalen via API
API Response: {id: 2, name: 'Ervin Howell', username: 'Antonette', email: 'Shanna@melissa.tv', address: {…}, …}
Component 3: ontvangt gebruikersdata {id: 2, name: 'Ervin Howell', username: 'Antonette', email: 'Shanna@melissa.tv', address: {…}, …}
```

## Ontwerppatronen van Multicasting

### Singleton Observable

Patroon om een enkele Observable te delen over de hele applicatie.

```ts
import { Subject } from 'rxjs';

// Globale status management voor hele applicatie
class AppState {
  // Singleton instance
  private static instance: AppState;

  // Globale notificatie stream
  private notificationsSubject = new Subject<string>();

  // Publieke Observable (read-only)
  readonly notifications$ = this.notificationsSubject.asObservable();

  // Singleton toegang
  static getInstance(): AppState {
    if (!AppState.instance) {
      AppState.instance = new AppState();
    }
    return AppState.instance;
  }

  // Methode om notificatie te verzenden
  notify(message: string): void {
    this.notificationsSubject.next(message);
  }
}

// Gebruiksvoorbeeld
const appState = AppState.getInstance();

// Monitor notificaties (vanuit meerdere componenten)
appState.notifications$.subscribe((msg) =>
  console.log('Component A:', msg)
);
appState.notifications$.subscribe((msg) =>
  console.log('Component B:', msg)
);

// Verzend notificatie
appState.notify('Systeemupdate beschikbaar');
```

#### Uitvoerresultaat
```ts
Component A: Systeemupdate beschikbaar
Component B: Systeemupdate beschikbaar
```

## Samenvatting

Multicasting is een belangrijke techniek om de efficiëntie en prestaties van RxJS-applicaties te verbeteren. De belangrijkste punten zijn:

- Multicasting maakt het mogelijk om één databron te delen met meerdere abonnees
- Kan worden geïmplementeerd met operators zoals `share()`, `shareReplay()`, `publish()`
- Kan dubbele API-requests vermijden en dure verwerkingen optimaliseren
- Nuttig voor state management en communicatie tussen componenten

Door de juiste multicast-strategie te kiezen, kun je de responsiviteit en efficiëntie van je applicatie verhogen terwijl je de hoeveelheid code vermindert en de onderhoudbaarheid verbetert.
