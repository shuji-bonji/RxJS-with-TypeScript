---
description: Dit gedeelte geeft een overzicht van ajax en fromFetch, Creation Functions voor HTTP-communicatie in RxJS, de verschillen tussen beide en richtlijnen voor het gebruik ervan.
---

# HTTP-communicatie Creation Functions

RxJS biedt Creation Functions om HTTP-communicatie als Observable te behandelen. Dit gedeelte beschrijft twee functies, `ajax()` en `fromFetch()`, in detail.

## Wat zijn HTTP-communicatie Creation Functions?

HTTP-communicatie Creation Functions zijn een reeks functies waarmee communicatie met externe API's en servers als een Observable-stream kan worden behandeld. Door deze functies te gebruiken kan asynchrone HTTP-communicatie worden geïntegreerd in de RxJS operator-keten, en kunnen foutafhandeling en retry-verwerking declaratief worden beschreven.

### Belangrijkste kenmerken

- **Declaratieve HTTP-communicatie**: Door HTTP-communicatie als Observable te behandelen, is declaratieve verwerking met operators mogelijk
- **Uniforme foutafhandeling**: Uniforme foutafhandeling met operators zoals `catchError()` en `retry()`
- **Annuleerbaar**: Verzoeken kunnen worden geannuleerd met `unsubscribe()`
- **Integratie met andere streams**: Combineren met andere Observables via `switchMap()`, enz.

## Lijst van HTTP-communicatie Creation Functions

| Functie | Beschrijving | Basistechnologie | Belangrijkste toepassingen |
|---------|--------------|------------------|---------------------------|
| [ajax()](/nl/guide/creation-functions/http-communication/ajax) | XMLHttpRequest-gebaseerde HTTP-communicatie | XMLHttpRequest | Ondersteuning voor legacy browsers, voortgangsmonitoring |
| [fromFetch()](/nl/guide/creation-functions/http-communication/fromFetch) | Fetch API-gebaseerde HTTP-communicatie | Fetch API | Moderne browsers, lichtgewicht HTTP-communicatie |

## Vergelijking: ajax() vs fromFetch()

### Basisverschillen

```typescript
import { switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';
import { fromFetch } from 'rxjs/fetch';

// ajax() - Parset automatisch het antwoord
const ajax$ = ajax.getJSON<Todo>('https://jsonplaceholder.typicode.com/todos/1');
ajax$.subscribe(data => console.log(data));

// fromFetch() - Handmatig antwoord parsen
const fetch$ = fromFetch('https://jsonplaceholder.typicode.com/todos/1').pipe(
  switchMap(response => response.json())
);
fetch$.subscribe(data => console.log(data));

interface Todo {
  userId: number;
  id: number;
  title: string;
  completed: boolean;
}
```

### Functievergelijkingstabel

| Functie | ajax() | fromFetch() |
|---------|--------|-------------|
| Basistechnologie | XMLHttpRequest | Fetch API |
| Automatische JSON-parsing | ✅ Ondersteund met `getJSON()` | ❌ Handmatig `.json()` aanroepen |
| Voortgangsgebeurtenissen | ✅ Ondersteund | ❌ Niet ondersteund |
| Timeout | ✅ Ingebouwde ondersteuning | ❌ Handmatige implementatie vereist |
| Automatische HTTP-foutdetectie | ✅ Automatisch fout bij 4xx/5xx | ❌ Handmatige statuscontrole vereist |
| Verzoek annuleren | ✅ Mogelijk met unsubscribe() | ✅ Mogelijk met unsubscribe() |
| IE11-ondersteuning | ✅ Ondersteund | ❌ Polyfill vereist |
| Bundelgrootte | Iets groter | Kleiner |

## Gebruiksrichtlijnen

### Wanneer ajax() te kiezen

1. **Ondersteuning voor legacy browsers is vereist**
   - Wanneer u oudere browsers zoals IE11 moet ondersteunen

2. **Voortgangsmonitoring is vereist**
   - Wanneer u de voortgang van bestandsupload/-download wilt weergeven

3. **Eenvoudige JSON-ophaling**
   - Wanneer u JSON gemakkelijk wilt ophalen met `getJSON()`

4. **Automatische foutdetectie is nodig**
   - Wanneer u automatische foutdetectie op basis van HTTP-statuscode wilt gebruiken

### Wanneer fromFetch() te kiezen

1. **Alleen moderne browsers worden ondersteund**
   - Wanneer u alleen omgevingen ondersteunt waar de Fetch API beschikbaar is

2. **Bundelgrootte verkleinen**
   - Wanneer een lichtgewicht HTTP-communicatiefunctie voldoende is

3. **Fetch API-functies gebruiken**
   - Wanneer u Request/Response-objecten direct wilt manipuleren
   - Wanneer u het wilt gebruiken in een Service Worker

4. **Fijne controle nodig**
   - Wanneer u de responsverwerking in detail wilt aanpassen

## Praktische gebruiksvoorbeelden

### API-aanroeppatroon

```typescript
import { of, catchError, retry, timeout } from 'rxjs';
import { ajax } from 'rxjs/ajax';

interface User {
  id: number;
  name: string;
  email: string;
}

// Praktisch patroon met ajax()
const fetchUser$ = ajax.getJSON<User>('https://api.example.com/users/1').pipe(
  timeout(5000), // Timeout na 5 seconden
  retry(2), // Twee keer opnieuw proberen bij falen
  catchError(error => {
    console.error('Fout bij ophalen gebruiker:', error);
    return of(null); // Retourneer null bij fout
  })
);

fetchUser$.subscribe({
  next: user => {
    if (user) {
      console.log('Gebruiker:', user);
    } else {
      console.log('Ophalen gebruiker mislukt');
    }
  }
});
```

### Formulierverzendingspatroon

```typescript
import { fromEvent, switchMap, map } from 'rxjs';
import { ajax } from 'rxjs/ajax';

// Converteer formulier submit event naar Observable
const form = document.querySelector('form') as HTMLFormElement;
const submit$ = fromEvent(form, 'submit').pipe(
  map(event => {
    event.preventDefault();
    const formData = new FormData(form);
    return Object.fromEntries(formData.entries());
  }),
  switchMap(data =>
    ajax.post('https://api.example.com/submit', data, {
      'Content-Type': 'application/json'
    })
  )
);

submit$.subscribe({
  next: response => console.log('Verzending geslaagd:', response),
  error: error => console.error('Verzendingsfout:', error)
});
```

## Veelgestelde vragen

### V1: Moet ik ajax() of fromFetch() gebruiken?

**A:** We raden `fromFetch()` aan als alleen moderne browsers worden ondersteund. De redenen zijn als volgt:
- Fetch API is de nieuwste webstandaard
- Kleine bundelgrootte
- Hoge toekomstige compatibiliteit

Kies echter `ajax()` in de volgende gevallen:
- IE11-ondersteuning is vereist
- Voortgangsmonitoring is vereist
- Eenvoudige JSON-ophaling is voldoende

### V2: Hoe worden HTTP-fouten (4xx, 5xx) afgehandeld?

**A:**
- **ajax()**: HTTP-statuscode boven 400 wordt automatisch als fout behandeld en de `error`-callback wordt aangeroepen
- **fromFetch()**: HTTP-fouten activeren nog steeds de `next`-callback. U moet `response.ok` handmatig controleren

### V3: Hoe annuleer ik een verzoek?

**A:** Beide kunnen worden geannuleerd met `unsubscribe()`.

```typescript
const subscription = ajax.getJSON('/api/data').subscribe(...);

// Annuleren na 3 seconden
setTimeout(() => subscription.unsubscribe(), 3000);
```

## Volgende stappen

Voor gedetailleerd gebruik van elke functie, raadpleeg de volgende pagina's:

- [ajax() in detail](/nl/guide/creation-functions/http-communication/ajax) - XMLHttpRequest-gebaseerde HTTP-communicatie
- [fromFetch() in detail](/nl/guide/creation-functions/http-communication/fromFetch) - Fetch API-gebaseerde HTTP-communicatie

## Referentiebronnen

- [RxJS Officiële Documentatie - ajax](https://rxjs.dev/api/ajax/ajax)
- [RxJS Officiële Documentatie - fromFetch](https://rxjs.dev/api/fetch/fromFetch)
- [MDN Web Docs - Fetch API](https://developer.mozilla.org/en-US/docs/Web/API/Fetch_API)
- [MDN Web Docs - XMLHttpRequest](https://developer.mozilla.org/en-US/docs/Web/API/XMLHttpRequest)
