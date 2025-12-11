---
description: "Begrijp het verschil tussen Promise en RxJS en leer de juiste keuze te maken. Promise is gespecialiseerd in enkele asynchrone verwerkingen en wordt onmiddellijk uitgevoerd, terwijl RxJS lazy-evaluatie streamverwerking biedt die meerdere waarden kan behandelen. We vergelijken annulering, opnieuw proberen en transformatie met TypeScript."
---

# Verschil tussen Promise en RxJS

## Overzicht

De belangrijkste tools voor het omgaan met asynchrone verwerking in JavaScript/TypeScript zijn **Promise** en **RxJS (Observable)**. Beide worden soms voor vergelijkbare doeleinden gebruikt, maar de ontwerpfilosofie en use cases zijn aanzienlijk verschillend.

Deze pagina helpt je het verschil tussen Promise en RxJS te begrijpen en te bepalen welke je moet gebruiken.

## Fundamentele verschillen

| Item | Promise | RxJS (Observable) |
|------|---------|-------------------|
| **Standaardisatie** | JavaScript-standaard (ES6/ES2015) | Third-party bibliotheek |
| **Uitgegeven waarden** | Enkele waarde | 0 of meer meerdere waarden |
| **Evaluatie** | Eager (onmiddellijke uitvoering bij aanmaak) | Lazy (uitvoering bij subscribe) |
| **Annuleren** | Niet mogelijk[^1] | Mogelijk (`unsubscribe()`) |
| **Hergebruik** | Niet mogelijk (resultaat slechts 1x) | Mogelijk (meerdere keren subscriben) |
| **Leercurve** | Laag | Hoog (begrip van operators vereist) |
| **Use cases** | Enkele asynchrone verwerking | Complexe streamverwerking |

[^1]: Met AbortController is het annuleren van Promise-gebaseerde verwerking (zoals fetch) mogelijk, maar Promise-specificatie zelf heeft geen annulatiefunctionaliteit.

## Code vergelijking: enkele asynchrone verwerking

### Promise

```ts
// Promise wordt onmiddellijk uitgevoerd bij aanmaak (Eager)
const promise = fetch('https://jsonplaceholder.typicode.com/posts/1')
  .then(response => response.json())
  .then(data => console.log(data))
  .catch(error => console.error(error));
```

Promise **begint onmiddellijk met uitvoeren zodra het is gedefinieerd** (Eager evaluatie).

### RxJS

```ts
import { from } from 'rxjs';
import { switchMap, catchError } from 'rxjs';
import { of } from 'rxjs';

// Observable wordt pas uitgevoerd bij subscribe (Lazy)
const observable$ = from(fetch('https://jsonplaceholder.typicode.com/posts/1')).pipe(
  switchMap(response => response.json()), // response.json() retourneert Promise dus we gebruiken switchMap
  catchError(error => {
    console.error(error);
    return of(null);
  })
);

// Wordt pas uitgevoerd bij subscribe
observable$.subscribe(data => console.log(data));
```

RxJS **wordt niet uitgevoerd totdat `subscribe()` wordt aangeroepen** (Lazy evaluatie). Als je meerdere keren op dezelfde Observable subscribet, worden onafhankelijke uitvoeringen uitgevoerd, en je kunt de verwerking onderbreken met `unsubscribe()`.

> [!TIP]
> **Praktische keuze**
> - Enkele verwerking die onmiddellijk moet worden uitgevoerd → Promise
> - Verwerking die op het juiste moment moet worden uitgevoerd, of meerdere keren moet worden uitgevoerd → RxJS

## Code vergelijking: omgaan met meerdere waarden

Een van de grootste verschillen tussen Promise en RxJS is het aantal waarden dat kan worden uitgegeven. Promise kan slechts één enkele waarde retourneren, maar RxJS kan meerdere waarden in de tijd uitgeven.

### Niet mogelijk met Promise

Promise kan **slechts eenmaal worden opgelost**.

```ts
// Promise kan slechts één enkele waarde retourneren
const promise = new Promise(resolve => {
  resolve(1);
  resolve(2); // Deze waarde wordt genegeerd
  resolve(3); // Deze waarde wordt ook genegeerd
});

promise.then(value => console.log(value));
// Output: 1 (alleen de eerste waarde)
```

Zodra de waarde is vastgelegd met de eerste `resolve()`, worden alle volgende `resolve()` genegeerd.

### Mogelijk met RxJS

Observable kan **meerdere keren waarden uitgeven**.
```ts
import { Observable } from 'rxjs';

// Observable kan meerdere waarden uitgeven
const observable$ = new Observable(subscriber => {
  subscriber.next(1);
  subscriber.next(2);
  subscriber.next(3);
  subscriber.complete();
});

observable$.subscribe(value => console.log(value));
// Output: 1, 2, 3
```

Telkens wanneer `next()` wordt aangeroepen, komt de waarde bij de subscriber aan. Nadat alle waarden zijn uitgegeven, wordt voltooiing gemeld met `complete()`. Deze eigenschap maakt het natuurlijk om tijdreeksgegevens zoals realtime communicatie, streaminggegevens en opeenvolgende event-afhandeling te behandelen.

> [!NOTE]
> **Praktische toepassingsvoorbeelden**
> - WebSocket-berichtreceptie
> - Opeenvolgende verwerking van toetsenbordinvoer
> - Event stream van server (SSE)
> - Continue monitoring van sensorgegevens

## Vergelijking van annulering

Of je langdurige verwerking of onnodige asynchrone verwerking kunt annuleren, is belangrijk vanuit het perspectief van resourcebeheer en gebruikerservaring. Er is een groot verschil in annulatiefunctionaliteit tussen Promise en RxJS.

### Promise (niet annuleerbaar)
Promise heeft **geen standaard annulatiefunctionaliteit**.

```ts
const promise = new Promise(resolve => {
  setTimeout(() => resolve('Voltooid'), 3000);
});

promise.then(result => console.log(result));
// Er is geen standaardmanier om deze verwerking te annuleren
```

Eenmaal begonnen, kan het niet worden gestopt totdat het is voltooid, wat kan leiden tot geheugenlekken en prestatievermindering.

> [!WARNING]
> **Over AbortController**
> Web API's zoals `fetch()` kunnen worden geannuleerd met `AbortController`, maar dit is geen functionaliteit van Promise zelf, maar een mechanisme dat wordt geboden door individuele API's. Het kan niet voor alle asynchrone verwerkingen worden gebruikt.

### RxJS (annuleerbaar)

RxJS kan **altijd worden geannuleerd met `unsubscribe()`**.
```ts
import { timer } from 'rxjs';

const subscription = timer(3000).subscribe(
  () => console.log('Voltooid')
);

// Annuleren na 1 seconde
setTimeout(() => {
  subscription.unsubscribe(); // Annuleren
  console.log('Geannuleerd');
}, 1000);
// Output: Geannuleerd ("Voltooid" wordt niet uitgevoerd)
```

Door het abonnement op te zeggen, stopt de lopende verwerking onmiddellijk en voorkom je geheugenlekken.

> [!TIP]
> **Praktische toepassingsvoorbeelden van annulering**
> - HTTP-verzoeken annuleren wanneer gebruiker het scherm verlaat
> - Oude zoekqueryresultaten weggooien en alleen de nieuwste query verwerken (`switchMap`)
> - Automatisch alle Observables annuleren bij vernietiging van component (`takeUntil`-patroon)

## Welke kiezen?

Welke van Promise of RxJS je moet gebruiken, hangt af van de aard van de verwerking en de vereisten van het project. Gebruik de volgende criteria als referentie om de juiste tool te selecteren.

### Wanneer Promise te kiezen

Als aan de volgende voorwaarden is voldaan, is Promise geschikt.

| Voorwaarde | Reden |
|------|------|
| Enkele asynchrone verwerking | 1 API-verzoek, 1 bestandslezen, etc. |
| Eenvoudige workflow | `Promise.all`, `Promise.race` zijn voldoende |
| Kleinschalig project | Wil afhankelijkheden minimaliseren |
| Alleen standaard API's gebruiken | Wil externe bibliotheken vermijden |
| Code voor beginners | Wil leercurve verlagen |

#### Enkel API-verzoek:


```ts
interface User {
  id: number;
  name: string;
  email: string;
  username: string;
}

async function getUserData(userId: string): Promise<User> {
  const response = await fetch(`https://jsonplaceholder.typicode.com/users/${userId}`);
  if (!response.ok) {
    throw new Error('Ophalen gebruikersgegevens mislukt');
  }
  return response.json();
}

// Gebruiksvoorbeeld
getUserData('1').then(user => {
  console.log('Gebruikersnaam:', user.name);
  console.log('E-mail:', user.email);
});
```

Deze code is een typisch patroon voor het ophalen van enkele gebruikersinformatie. Door `async/await` te gebruiken, kun je het leesbaar schrijven alsof het synchrone code is. Foutafhandeling kan ook uniform worden gedaan met `try/catch`, eenvoudig en intuïtief.

#### Parallel uitvoeren van meerdere asynchrone verwerkingen:

```ts
interface Post {
  id: number;
  userId: number;
  title: string;
  body: string;
}

async function loadAllData(): Promise<[User[], Post[]]> {
  const [users, posts] = await Promise.all([
    fetch('https://jsonplaceholder.typicode.com/users').then(r => r.json()),
    fetch('https://jsonplaceholder.typicode.com/posts').then(r => r.json())
  ]);
  return [users, posts];
}

// Gebruiksvoorbeeld
loadAllData().then(([users, posts]) => {
  console.log('Aantal gebruikers:', users.length);
  console.log('Aantal posts:', posts.length);
});
```

Door `Promise.all()` te gebruiken, kun je meerdere API-verzoeken parallel uitvoeren en wachten tot ze allemaal zijn voltooid. Dit is zeer handig voor het laden van initiële gegevens. Let op dat als er één mislukt, het hele geheel een fout geeft, maar juist door die eenvoud is het gemakkelijk te begrijpen en te onderhouden.

### Wanneer RxJS te kiezen

Als aan de volgende voorwaarden is voldaan, is RxJS geschikt.

| Voorwaarde | Reden |
|------|------|
| Opeenvolgende event-afhandeling | Muisbewegingen, toetsenbordinvoer, WebSocket, etc. |
| Complexe streamverwerking | Combineren en transformeren van meerdere eventbronnen |
| Annulering nodig | Wil resourcebeheer gedetailleerd controleren |
| Opnieuw proberen & timeout | Wil foutafhandeling flexibel uitvoeren |
| Angular-project | RxJS is geïntegreerd in framework |
| Realtime gegevens | Gegevens worden continu bijgewerkt |

#### Concreet voorbeeld
```ts
import { fromEvent } from 'rxjs';
import { debounceTime, map, distinctUntilChanged, switchMap } from 'rxjs';

const label = document.createElement('label');
label.innerText = 'zoeken: ';
const searchInput = document.createElement('input');
searchInput.type = 'input';
label.appendChild(searchInput);
document.body.appendChild(label);

// Realtime zoeken (autocomplete)
if (!searchInput) throw new Error('Zoekinvoerveld niet gevonden');

fromEvent(searchInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value),
  debounceTime(300),              // Wacht 300ms voordat verwerken
  distinctUntilChanged(),         // Verwerk alleen bij waardewijziging
  switchMap(query =>              // Voer alleen meest recente verzoek uit
    fetch(`https://api.github.com/search/users?q=${query}`).then(r => r.json())
  )
).subscribe(results => {
  console.log('Zoekresultaten:', results.items); // GitHub API slaat resultaten op in items-property
});
```

Dit voorbeeld is een typisch geval waar de ware waarde van RxJS wordt getoond. Het monitort gebruikersinvoer, vermindert onnodige verzoeken door een wachttijd van 300ms in te stellen, verwerkt alleen wanneer de waarde verandert, en maakt bovendien alleen het nieuwste verzoek geldig (`switchMap`), waardoor resultaten van oude verzoeken automatisch worden weggegooid.

> [!IMPORTANT]
> **Waarom moeilijk met alleen Promise**
> - Debounce (controle van continue invoer) moet handmatig worden geïmplementeerd
> - Je moet het annuleren van oude verzoeken zelf beheren
> - Als je vergeet event listeners op te ruimen, ontstaan geheugenlekken
> - Je moet meerdere statussen (timer, flags, verzoekbeheer) tegelijkertijd bijhouden
>
> Met RxJS kan dit allemaal declaratief in enkele regels worden gerealiseerd.

## Interoperabiliteit tussen Promise en RxJS

Promise en RxJS zijn niet exclusief en kunnen worden omgezet en gecombineerd. Handig wanneer je bestaande Promise-gebaseerde code wilt integreren in een RxJS-pipeline, of omgekeerd een Observable wilt gebruiken in bestaande Promise-gebaseerde code.

## Promise naar Observable converteren

RxJS biedt meerdere manieren om een bestaande Promise naar Observable te converteren.

### Conversie met `from`

De meest gebruikelijke methode is het gebruik van `from`.

```ts
import { from } from 'rxjs';

// Promise aanmaken
const promise = fetch('https://jsonplaceholder.typicode.com/posts/1')
  .then(response => response.json());

// Converteren naar Observable met from()
const observable$ = from(promise);

observable$.subscribe({
  next: data => console.log('Gegevens:', data),
  error: error => console.error('Fout:', error),
  complete: () => console.log('Voltooid')
});
```

`from()` geeft 1 waarde uit wanneer de Promise wordt opgelost en voert onmiddellijk `complete` uit. Als er een fout optreedt, wordt een `error`-notificatie verzonden. Door deze conversie kun je RxJS-operators (`map`, `filter`, `retry`, etc.) vrij toepassen op gegevens afkomstig van Promise.

### Conversie met `defer` (luie evaluatie)

`defer` stelt het aanmaken van de Promise uit tot subscribe.

```ts
import { defer } from 'rxjs';

// Promise wordt niet aangemaakt totdat subscribe wordt uitgevoerd
const observable$ = defer(() =>
  fetch('https://jsonplaceholder.typicode.com/posts/1').then(r => r.json())
);

// Maakt nieuwe Promise aan per subscribe
observable$.subscribe(data => console.log('1e keer:', data));
observable$.subscribe(data => console.log('2e keer:', data));
```

Deze methode is handig wanneer je bij elke subscribe een nieuwe Promise wilt aanmaken.

## Observable naar Promise converteren

Je kunt slechts één waarde uit een Observable halen en er een Promise van maken.

### `firstValueFrom` en `lastValueFrom`

Vanaf RxJS 7 worden de volgende twee functies aanbevolen.

| Functie | Werking |
|------|------|
| `firstValueFrom` | Retourneert eerste waarde als Promise |
| `lastValueFrom` | Retourneert laatste waarde bij voltooiing als Promise |

```ts
import { of, firstValueFrom, lastValueFrom } from 'rxjs';
import { delay } from 'rxjs';

const observable$ = of(1, 2, 3).pipe(delay(1000));

// Eerste waarde als Promise ophalen
const firstValue = await firstValueFrom(observable$);
console.log(firstValue); // 1

// Laatste waarde als Promise ophalen
const lastValue = await lastValueFrom(observable$);
console.log(lastValue); // 3
```

Als Observable voltooit voordat waarden worden doorgegeven, resulteert dit standaard in een fout. Dit kan worden voorkomen door een standaardwaarde op te geven.

> [!WARNING]
> `toPromise()` is verouderd. Gebruik in plaats daarvan `firstValueFrom()` of `lastValueFrom()`.

> [!TIP]
> **Selectierichtlijn**
> - **`firstValueFrom()`**: Wanneer alleen de eerste waarde nodig is (bijv: login authenticatieresultaat)
> - **`lastValueFrom()`**: Wanneer eindresultaat na verwerking van alle gegevens nodig is (bijv: aggregatieresultaat)

## Praktisch voorbeeld: beide combineren

In echte applicaties is het gebruikelijk om Promise en RxJS gecombineerd te gebruiken.

> [!WARNING] Aandachtspunten in de praktijk
> Het mengen van Promise en Observable kan **gemakkelijk leiden tot anti-patterns als de ontwerpgrenzen niet duidelijk zijn**.
>
> **Veelvoorkomende problemen:**
> - Wordt niet-annuleerbaar
> - Scheiding van foutafhandeling
> - `await` binnen `subscribe` (bijzonder gevaarlijk)
> - Parallel ophalen van dezelfde gegevens met Promise en Observable
>
> Zie voor details **[Hoofdstuk 10: Anti-pattern van Promise en Observable mengen](/nl/guide/anti-patterns/promise-observable-mixing)**.

### Formulierinzending en API-aanroep

Voorbeeld van het opvangen van gebruikersformulier-inzendings-events met RxJS en verzenden naar server met Fetch API (Promise).

```ts
import { fromEvent, from } from 'rxjs';
import { exhaustMap, catchError } from 'rxjs';
import { of } from 'rxjs';

interface FormData {
  username: string;
  email: string;
}

// Promise-gebaseerde formulierinzending
async function submitForm(data: FormData): Promise<{ success: boolean }> {
  const response = await fetch('https://api.example.com/submit', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(data)
  });
  if (!response.ok) {
    throw new Error('Verzenden mislukt');
  }
  return response.json();
}

// Eventstream beheren met RxJS
const submitButton = document.createElement('button');
submitButton.id = 'submit-button';
submitButton.innerText = 'Verzenden';
submitButton.style.padding = '10px 20px';
submitButton.style.margin = '10px';
document.body.appendChild(submitButton);
if (!submitButton) throw new Error('Verzendknop niet gevonden');

fromEvent(submitButton, 'click').pipe(
  exhaustMap(() => {
    const formData: FormData = {
      username: 'testuser',
      email: 'test@example.com'
    };
    // Promise-functie converteren naar Observable
    return from(submitForm(formData));
  }),
  catchError(error => {
    console.error('Verzendfout:', error);
    return of({ success: false });
  })
).subscribe(result => {
  if (result.success) {
    console.log('Verzending geslaagd');
  } else {
    console.log('Verzending mislukt');
  }
});
```

Telkens wanneer de formulier-verzendknop wordt geklikt, wordt een nieuw verzendproces gestart, maar **nieuwe verzendingen worden genegeerd tijdens het verzenden**.

In dit voorbeeld voorkomt het gebruik van `exhaustMap` dubbele verzoeken tijdens verzending.

### Zoek-autocomplete

Voorbeeld van het monitoren van wijzigingen in invoerformulierwaarde en uitvoeren van API-zoekactie.

```ts
import { fromEvent, from } from 'rxjs';
import { debounceTime, switchMap, catchError } from 'rxjs';
import { of } from 'rxjs';

interface SearchResult {
  items: Array<{
    login: string;
    id: number;
    avatar_url: string;
  }>;
  total_count: number;
}

// Promise-gebaseerde API-functie
async function searchAPI(query: string): Promise<SearchResult> {
  const response = await fetch(`https://api.github.com/search/users?q=${query}`);
  if (!response.ok) {
    throw new Error('Zoeken mislukt');
  }
  return response.json();
}

// Eventstream beheren met RxJS
const label = document.createElement('label');
label.innerText = 'zoeken: ';
const searchInput = document.createElement('input');
searchInput.type = 'input';
label.appendChild(searchInput);
document.body.appendChild(label);
if (!searchInput) throw new Error('Zoekinvoerveld niet gevonden');

fromEvent(searchInput, 'input').pipe(
  debounceTime(300),
  switchMap(event => {
    const query = (event.target as HTMLInputElement).value;
    // Promise-functie converteren naar Observable
    return from(searchAPI(query));
  }),
  catchError(error => {
    console.error(error);
    return of({ items: [], total_count: 0 }); // Bij fout lege resultaten retourneren
  })
).subscribe(result => {
  console.log('Zoekresultaten:', result.items);
  console.log('Totaal:', result.total_count);
});
```

> [!TIP]
> **Ontwerp door scheiding van verantwoordelijkheden**
>
> - **RxJS**: Verantwoordelijk voor event-controle (debounce, switchMap, etc.)
> - **Promise**: Verantwoordelijk voor HTTP-verzoeken (async/await)
> - **`from()`**: Overbrugt beide
>
> Door elke technologie op de juiste plaats te gebruiken, verbeteren de leesbaarheid en onderhoudbaarheid van de code.

## Voor- en nadelen

### Promise
<div class="comparison-cards">

::: tip Voordelen
- Geen afhankelijkheden nodig omdat het een JavaScript-standaard is
- Intuïtieve en leesbare code dankzij `async/await`
- Lage leercurve
- Eenvoudige verwerking van enkele taken
:::

::: danger Nadelen
- Kan niet omgaan met meerdere waarden
- Geen annulatiefunctionaliteit
- Niet geschikt voor opeenvolgende streamverwerking
- Moeilijke complexe event-afhandeling
:::

</div>

### RxJS
<div class="comparison-cards">

::: tip Voordelen
- Kan meerdere waarden in tijdreeks behandelen
- Complexe verwerking mogelijk met rijke operators
- Annuleren (`unsubscribe`) is gemakkelijk
- Flexibele implementatie van foutafhandeling en opnieuw proberen mogelijk
- Declaratief en gemakkelijk te testen
:::

::: danger Nadelen
- Hoge leercurve
- Afhankelijkheid van bibliotheek nodig
- Heeft overhead (excessief voor kleinschalige projecten)
- Debuggen kan moeilijk zijn
:::

</div>

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

## Gebieden waar RxJS bijzonder uitblinkt

RxJS is bijzonder krachtig in de volgende gebieden. Het kan complexe vereisten die moeilijk te realiseren zijn met alleen Promise, elegant oplossen.

| Gebied | Concreet voorbeeld | Vergelijking met Promise |
|------|--------|----------------|
| **Realtime communicatie** | WebSocket, SSE, chat, koersupdates | Promise alleen voor eenmalige communicatie. Niet geschikt voor opeenvolgende berichtverwerking |
| **Gebruikersinvoer-controle** | Zoek-autocomplete, formuliervalidatie | debounce, distinctUntilChanged etc. standaard uitgerust |
| **Combinatie van meerdere bronnen** | Combinatie van zoekvoorwaarden × sorteervolgorde × filters | Beknopt beschreven met combineLatest, withLatestFrom |
| **Offline-ondersteuning** | PWA, netwerkstatusmonitoring, automatische hersynchronisatie | Flexibele retry-controle met retry, retryWhen |
| **Streaming-API's** | OpenAI, AI-respons token-voor-token uitvoer | Continue gegevens realtime verwerken mogelijk |
| **Annulatie-controle** | Onderbreken van langdurige verwerking, weggooien oude verzoeken | Onmiddellijk annuleren mogelijk met unsubscribe() |

> [!NOTE]
> Zie ook [Wat is RxJS - Use cases](./what-is-rxjs.md#use-cases) voor details over toepassingsgebieden van RxJS.

## Samenvatting

| Doel | Aanbevolen | Reden |
|------|------|------|
| Enkel HTTP-verzoek | Promise (`async/await`) | Eenvoudig en leesbaar, standaard API |
| Verwerking van gebruikersinvoer-events | RxJS | Controle zoals debounce, distinct nodig |
| Realtime gegevens (WebSocket) | RxJS | Kan opeenvolgende berichten natuurlijk behandelen |
| Parallel uitvoeren van meerdere asynchrone verwerkingen | Promise (`Promise.all`) | Promise is voldoende voor eenvoudige parallelle uitvoering |
| Opeenvolgende eventstream | RxJS | Kan meerdere waarden in tijdreeks behandelen |
| Annuleerbare verwerking | RxJS | Betrouwbaar annuleren met unsubscribe() |
| Eenvoudige applicatie | Promise | Lage leercurve, weinig afhankelijkheden |
| Angular-applicatie | RxJS | Standaard geïntegreerd in framework |

### Basisbeleid
- **Gebruik Promise als het eenvoudig kan**
- **Gebruik RxJS als complexe streamverwerking nodig is**
- **Combineren van beide** is ook effectief (overbruggen met `from()`)

RxJS is krachtig, maar je hoeft niet RxJS te gebruiken voor alle asynchrone verwerkingen. Het is belangrijk om de juiste tool op de juiste plaats te gebruiken.

> [!TIP] Volgende stap
> - Leer details van Observable in [Wat is Observable](/nl/guide/observables/what-is-observable)
> - Leer hoe Observable aan te maken in [Creation Functions](/nl/guide/creation-functions/index)
> - Leer transformatie en controle van Observable in [Operators](/nl/guide/operators/index)
