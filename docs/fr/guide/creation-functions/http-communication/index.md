---
description: "Cette section fournit une vue d'ensemble de ajax et fromFetch, les fonctions de création pour la communication HTTP dans RxJS, les différences entre eux, et les lignes directrices pour leur utilisation."
---

# Fonctions de création pour la communication HTTP

RxJS fournit des fonctions de création pour gérer la communication HTTP en tant qu'Observable. Cette section décrit en détail deux fonctions, `ajax()` et `fromFetch()`.

## Que sont les fonctions de création de communication HTTP ?

Les fonctions de création de communication HTTP sont un ensemble de fonctions qui permettent de gérer la communication avec des API et des serveurs externes en tant que flux Observable. En utilisant ces fonctions, la communication HTTP asynchrone peut être intégrée dans la chaîne d'opérateurs RxJS, et la gestion des erreurs et le traitement des tentatives peuvent être décrits de manière déclarative.

### Caractéristiques principales

- **Communication HTTP déclarative** : En traitant la communication HTTP comme un Observable, le traitement déclaratif à l'aide d'opérateurs est possible
- **Traitement uniforme des erreurs** : Traitement uniforme des erreurs à l'aide d'opérateurs tels que `catchError()` et `retry()`
- **Annulable** : Les requêtes peuvent être annulées avec `unsubscribe()`
- **Intégration avec d'autres flux** : Combinaison avec d'autres Observables via `switchMap()`, etc.

## Liste des fonctions de création de communication HTTP

| Fonction | Description | Technologie de base | Principales utilisations |
|----------|-------------|---------------------|--------------------------|
| [ajax()](/fr/guide/creation-functions/http-communication/ajax) | Communication HTTP basée sur XMLHttpRequest | XMLHttpRequest | Prise en charge des navigateurs anciens, suivi de la progression |
| [fromFetch()](/fr/guide/creation-functions/http-communication/fromFetch) | Communication HTTP basée sur Fetch API | Fetch API | Navigateurs modernes, communication HTTP légère |

## Comparaison : ajax() vs fromFetch()

### Différences fondamentales

```typescript
import { switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';
import { fromFetch } from 'rxjs/fetch';

// ajax() - Analyse automatiquement la réponse
const ajax$ = ajax.getJSON<Todo>('https://jsonplaceholder.typicode.com/todos/1');
ajax$.subscribe(data => console.log(data));

// fromFetch() - Analyser manuellement la réponse
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

### Tableau de comparaison des fonctionnalités

| Fonctionnalité | ajax() | fromFetch() |
|----------------|--------|-------------|
| Technologie de base | XMLHttpRequest | Fetch API |
| Analyse JSON automatique | ✅ Pris en charge par `getJSON()` | ❌ Appel manuel de `.json()` |
| Événements de progression | ✅ Pris en charge | ❌ Non pris en charge |
| Timeout | ✅ Support intégré | ❌ Implémentation manuelle requise |
| Détection automatique des erreurs HTTP | ✅ Erreurs automatiques sur 4xx/5xx | ❌ Vérification manuelle de l'état requise |
| Annulation de requête | ✅ Possible avec unsubscribe() | ✅ Possible avec unsubscribe() |
| Support IE11 | ✅ Pris en charge | ❌ Polyfill requis |
| Taille du bundle | Un peu plus grande | Plus petite |

## Directives d'utilisation

### Quand choisir ajax()

1. **La prise en charge des navigateurs anciens est requise**
   - Lorsque vous devez prendre en charge des navigateurs plus anciens tels que IE11

2. **Le suivi de la progression est nécessaire**
   - Lorsque vous souhaitez afficher la progression du téléchargement de fichiers

3. **Récupération simple de JSON**
   - Lorsque vous voulez obtenir JSON facilement avec `getJSON()`

4. **La détection automatique des erreurs est nécessaire**
   - Lorsque vous souhaitez utiliser la détection automatique d'erreur par le code d'état HTTP

### Quand choisir fromFetch()

1. **Seuls les navigateurs modernes sont pris en charge**
   - Lorsque vous ne prenez en charge que les environnements où l'API Fetch est disponible

2. **Vous souhaitez réduire la taille du bundle**
   - Lorsqu'une fonction de communication HTTP légère est suffisante

3. **Vous souhaitez utiliser les fonctionnalités de l'API Fetch**
   - Lorsque vous souhaitez manipuler directement les objets Request/Response
   - Lorsque vous voulez l'utiliser dans un Service Worker

4. **Vous avez besoin d'un contrôle précis**
   - Lorsque vous voulez personnaliser le traitement de la réponse en détail

## Exemples d'utilisation pratique

### Modèle d'appel API

```typescript
import { of, catchError, retry, timeout } from 'rxjs';
import { ajax } from 'rxjs/ajax';

interface User {
  id: number;
  name: string;
  email: string;
}

// Modèle pratique utilisant ajax()
const fetchUser$ = ajax.getJSON<User>('https://api.example.com/users/1').pipe(
  timeout(5000), // Timeout après 5 secondes
  retry(2), // Réessayer deux fois en cas d'échec
  catchError(error => {
    console.error('Erreur de récupération utilisateur:', error);
    return of(null); // Retourner null en cas d'erreur
  })
);

fetchUser$.subscribe({
  next: user => {
    if (user) {
      console.log('Utilisateur:', user);
    } else {
      console.log('Échec de la récupération de l\'utilisateur');
    }
  }
});
```

### Modèle de soumission de formulaire

```typescript
import { fromEvent, switchMap, map } from 'rxjs';
import { ajax } from 'rxjs/ajax';

// Convertir l'événement de soumission de formulaire en Observable
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
  next: response => console.log('Soumission réussie:', response),
  error: error => console.error('Erreur de soumission:', error)
});
```

## Foire aux questions

### Q1 : Dois-je utiliser ajax() ou fromFetch() ?

**R:** Nous recommandons `fromFetch()` si seuls les navigateurs modernes sont supportés. Les raisons sont les suivantes :
- L'API Fetch est la dernière norme Web
- Taille réduite du bundle
- Compatibilité future élevée

Cependant, choisissez `ajax()` dans les cas suivants :
- La prise en charge d'IE11 est nécessaire
- Le suivi de la progression est nécessaire
- Une simple récupération JSON est suffisante

### Q2 : Comment sont gérées les erreurs HTTP (4xx, 5xx) ?

**R:**
- **ajax()** : Le code d'état HTTP supérieur à 400 est automatiquement traité comme une erreur et le callback `error` est appelé
- **fromFetch()** : Les erreurs HTTP déclenchent toujours le callback `next`. Vous devez vérifier manuellement `response.ok`

### Q3 : Comment annuler une requête ?

**R:** Les deux peuvent être annulées avec `unsubscribe()`.

```typescript
const subscription = ajax.getJSON('/api/data').subscribe(...);

// Annuler après 3 secondes
setTimeout(() => subscription.unsubscribe(), 3000);
```

## Prochaines étapes

Pour une utilisation détaillée de chaque fonction, veuillez vous référer aux pages suivantes :

- [ajax() en détail](/fr/guide/creation-functions/http-communication/ajax) - Communication HTTP basée sur XMLHttpRequest
- [fromFetch() en détail](/fr/guide/creation-functions/http-communication/fromFetch) - Communication HTTP basée sur l'API Fetch

## Ressources de référence

- [Documentation officielle de RxJS - ajax](https://rxjs.dev/api/ajax/ajax)
- [Documentation officielle de RxJS - fromFetch](https://rxjs.dev/api/fetch/fromFetch)
- [MDN Web Docs - Fetch API](https://developer.mozilla.org/en-US/docs/Web/API/Fetch_API)
- [MDN Web Docs - XMLHttpRequest](https://developer.mozilla.org/en-US/docs/Web/API/XMLHttpRequest)
