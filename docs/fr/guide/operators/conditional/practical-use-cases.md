---
description: "Explique les cas pratiques d'utilisation des opérateurs conditionnels de RxJS (iif, defer), y compris le traitement de repli de l'API, les stratégies de cache, la sélection dynamique de la source de données, l'évaluation paresseuse basée sur la condition. Des modèles d'utilisation spécifiques dans des situations où des branchements de traitement dynamiques sont nécessaires avec des exemples de code TypeScript. Apprenez des modèles de mise en œuvre applicables immédiatement au développement d'applications réelles."
---

# Cas d'utilisation pratiques

En utilisant les opérateurs conditionnels de RxJS, il est possible de brancher et de basculer entre les flux en fonction d'états dynamiques.
Dans ce chapitre, vous pouvez expérimenter les modèles d'utilisation de chaque opérateur à travers du code fonctionnel avec interface utilisateur.

## Sélectionner différentes sources de données basées sur des conditions

```ts
import { iif, of, EMPTY } from 'rxjs';
import { switchMap, tap, catchError, retry } from 'rxjs';

// Création de l'interface utilisateur
const appContainer = document.createElement('div');
appContainer.innerHTML = '<h3>Application de sélection de source de données :</h3>';
document.body.appendChild(appContainer);

// Sélection d'options
const optionsDiv = document.createElement('div');
optionsDiv.style.marginBottom = '15px';
appContainer.appendChild(optionsDiv);

// Case à cocher (mode hors ligne)
const offlineCheck = document.createElement('input');
offlineCheck.type = 'checkbox';
offlineCheck.id = 'offlineMode';
optionsDiv.appendChild(offlineCheck);

const offlineLabel = document.createElement('label');
offlineLabel.htmlFor = 'offlineMode';
offlineLabel.textContent = 'Mode hors ligne';
offlineLabel.style.marginLeft = '5px';
optionsDiv.appendChild(offlineLabel);

// Entrée ID de recherche
const idInput = document.createElement('input');
idInput.type = 'number';
idInput.placeholder = 'ID (1-10)';
idInput.min = '1';
idInput.max = '10';
idInput.value = '1';
idInput.style.marginLeft = '15px';
idInput.style.width = '80px';
optionsDiv.appendChild(idInput);

// Bouton de recherche
const searchButton = document.createElement('button');
searchButton.textContent = 'Rechercher';
searchButton.style.marginLeft = '10px';
optionsDiv.appendChild(searchButton);

// Zone de résultats
const resultsArea = document.createElement('div');
resultsArea.style.padding = '15px';
resultsArea.style.border = '1px solid #ddd';
resultsArea.style.borderRadius = '5px';
resultsArea.style.backgroundColor = '#f9f9f9';
resultsArea.style.minHeight = '150px';
appContainer.appendChild(resultsArea);

type User = {
  lastUpdated?: Date;
  fromCache?: boolean;
  id: number;
  name: string;
  email: string;
};
type ErrorResult = {
  error: boolean;
  message: string;
};

// Données hors ligne (cache)
const cachedData: Record<number, User> = {
  1: { id: 1, name: 'Taro Yamada', email: 'yamada@example.com' },
  2: { id: 2, name: 'Hanako Sato', email: 'sato@example.com' },
  3: { id: 3, name: 'Ichiro Suzuki', email: 'suzuki@example.com' },
};

// Obtenir les données actuelles de l'API en ligne (JSONPlaceholder)
function fetchUserFromApi(id: number) {
  console.log(`Récupération de l'utilisateur ID ${id} depuis l'API...`);

  // Point de terminaison API réel
  const apiUrl = `https://jsonplaceholder.typicode.com/users/${id}`;

  return of(null).pipe(
    switchMap(() =>
      fetch(apiUrl).then((response) => {
        if (!response.ok) {
          throw new Error(`Erreur HTTP : ${response.status}`);
        }
        return response.json();
      })
    ),
    tap(() => console.log('Appel API réussi')),
    catchError((err) => {
      console.error('Échec de l\'appel API :', err);
      throw new Error('La requête API a échoué');
    })
  );
}

// Récupérer l'utilisateur depuis le cache
function getUserFromCache(id: number) {
  console.log(`Récupération de l'utilisateur ID ${id} depuis le cache...`);

  return iif(
    () => id in cachedData,
    of({ ...cachedData[id], fromCache: true }),
    EMPTY.pipe(
      tap(() => {
        throw new Error('Utilisateur non trouvé dans le cache');
      })
    )
  );
}

// Clic sur le bouton de recherche
searchButton.addEventListener('click', () => {
  const id = parseInt(idInput.value, 10);
  const isOffline = offlineCheck.checked;

  // Validation de l'entrée
  if (isNaN(id) || id < 1 || id > 10) {
    resultsArea.innerHTML =
      '<p style="color: red;">Veuillez saisir un ID valide (1-10)</p>';
    return;
  }

  // Affichage du chargement
  resultsArea.innerHTML = '<p>Récupération des données...</p>';

  // Sélection de la source de données basée sur le mode hors ligne
  iif(
    () => isOffline,
    getUserFromCache(id).pipe(
      catchError((err) => {
        console.error('Erreur de cache :', err);
        return of({ error: err.message });
      })
    ),
    fetchUserFromApi(id).pipe(
      retry(2), // Réessayer jusqu'à 2 fois
      catchError((err) => {
        console.error('Erreur API :', err);

        // Utiliser le cache comme solution de repli en cas d'échec de l'API
        return getUserFromCache(id).pipe(
          catchError(() =>
            of({ error: 'L\'API en ligne et le cache ont tous deux échoué' })
          )
        );
      })
    )
  ).subscribe({
    next: (result: any) => {
      if ('error' in result) {
        resultsArea.innerHTML = `<p style="color: red;">Erreur : ${result.message}</p>`;
      } else {
        const source = result.fromCache
          ? '<span style="color: orange;">(depuis le cache)</span>'
          : '<span style="color: green;">(depuis l\'API)</span>';

        resultsArea.innerHTML = `
          <h4>Informations utilisateur ${source}</h4>
          <p><strong>ID :</strong> ${result.id}</p>
          <p><strong>Nom :</strong> ${result.name}</p>
          <p><strong>Email :</strong> ${result.email}</p>
          ${
            result.lastUpdated
              ? `<p><small>Dernière mise à jour : ${new Date(
                  result.lastUpdated
                ).toLocaleString()}</small></p>`
              : ''
          }
        `;
      }
    },
    error: (err) => {
      resultsArea.innerHTML = `<p style="color: red;">Erreur : ${err.message}</p>`;
    },
  });
});

// Message initial
resultsArea.innerHTML = '<p>Cliquez sur le bouton pour récupérer les données</p>';


```



## Branchements et stratégies de repli en cours d'exécution

Cet exemple utilise `iif` pour basculer dynamiquement la source de données de « cache hors ligne » à « API en ligne » en fonction de l'interaction et de l'état de l'utilisateur.
De plus, en combinant `catchError` et `retry`, des tentatives et des destinations de repli peuvent être définies en cas d'échec.

Il est particulièrement adapté aux cas d'utilisation suivants :

- Prise en charge hors ligne dans des environnements réseau instables
- Utilisation du cache et basculement entre les requêtes en ligne
- Réessais automatiques et passage à des routes alternatives en cas d'échec de l'API

## Modèles d'optimisation des performances

Dans des scénarios plus complexes, des modèles optimisés de récupération de données peuvent être mis en œuvre en combinant des opérateurs conditionnels.

```ts
import { fromEvent, Observable, of, throwError, timer } from 'rxjs';
import {
  switchMap,
  catchError,
  map,
  tap,
  debounceTime,
  distinctUntilChanged,
  withLatestFrom,
  delay,
  startWith,
} from 'rxjs';

// Création des éléments d'interface utilisateur
const optimizationContainer = document.createElement('div');
optimizationContainer.innerHTML = '<h3>Récupération de données conditionnelle avancée :</h3>';
document.body.appendChild(optimizationContainer);

// Interface de recherche
const searchInputGroup = document.createElement('div');
searchInputGroup.style.marginBottom = '15px';
optimizationContainer.appendChild(searchInputGroup);

const searchInput = document.createElement('input');
searchInput.type = 'text';
searchInput.placeholder = 'Entrez l\'ID utilisateur (1-10)';
searchInput.value = '1';
searchInput.style.padding = '8px';
searchInput.style.width = '180px';
searchInputGroup.appendChild(searchInput);

const searchButton = document.createElement('button');
searchButton.textContent = 'Rechercher';
searchButton.style.marginLeft = '10px';
searchButton.style.padding = '8px 16px';
searchInputGroup.appendChild(searchButton);

// Paramétrage des options
const optionsGroup = document.createElement('div');
optionsGroup.style.marginBottom = '15px';
optimizationContainer.appendChild(optionsGroup);

const cacheCheckbox = document.createElement('input');
cacheCheckbox.type = 'checkbox';
cacheCheckbox.id = 'useCache';
cacheCheckbox.checked = true;
optionsGroup.appendChild(cacheCheckbox);

const cacheLabel = document.createElement('label');
cacheLabel.htmlFor = 'useCache';
cacheLabel.textContent = 'Utiliser le cache';
cacheLabel.style.marginRight = '15px';
optionsGroup.appendChild(cacheLabel);

const forceCheckbox = document.createElement('input');
forceCheckbox.type = 'checkbox';
forceCheckbox.id = 'forceRefresh';
optionsGroup.appendChild(forceCheckbox);

const forceLabel = document.createElement('label');
forceLabel.htmlFor = 'forceRefresh';
forceLabel.textContent = 'Forcer le rechargement';
optionsGroup.appendChild(forceLabel);

// Zone d'affichage des résultats
const optimizedResults = document.createElement('div');
optimizedResults.style.padding = '15px';
optimizedResults.style.border = '1px solid #ddd';
optimizedResults.style.borderRadius = '5px';
optimizedResults.style.minHeight = '150px';
optimizedResults.style.backgroundColor = '#f9f9f9';
optimizationContainer.appendChild(optimizedResults);

// Gestion du cache
const cache = new Map<string, { data: any; timestamp: number }>();
const CACHE_EXPIRY = 30000; // 30 secondes

// Obtenir les données utilisateur depuis l'API réelle (JSONPlaceholder)
function fetchUserData(id: string, forceRefresh: boolean): Observable<any> {
  // ID invalide
  if (!id || isNaN(Number(id)) || Number(id) < 1 || Number(id) > 10) {
    return throwError(
      () => new Error('ID utilisateur invalide : entrez un nombre entre 1 et 10')
    );
  }

  const cacheKey = `user-${id}`;
  const cachedItem = cache.get(cacheKey);
  const now = Date.now();

  // Vérification du cache (si dans le délai et que le rechargement n'est pas forcé)
  if (
    !forceRefresh &&
    cachedItem &&
    now - cachedItem.timestamp < CACHE_EXPIRY
  ) {
    console.log(`Récupération depuis le cache : ${id}`);
    return of({
      ...cachedItem.data,
      fromCache: true,
    }).pipe(delay(100)); // Simuler une réponse rapide
  }

  // Requête API réelle (JSONPlaceholder)
  console.log(`Récupération des données depuis l'API : ${id}`);
  const apiUrl = `https://jsonplaceholder.typicode.com/users/${id}`;

  return of(null).pipe(
    switchMap(() =>
      fetch(apiUrl).then((response) => {
        if (!response.ok) {
          throw new Error(`Erreur HTTP : ${response.status}`);
        }
        return response.json();
      })
    ),
    map((userData) => {
      const processedData = {
        id: userData.id,
        name: userData.name,
        email: userData.email,
        lastUpdated: now,
        fromCache: false,
      };

      // Stockage dans le cache
      cache.set(cacheKey, {
        data: processedData,
        timestamp: now,
      });

      return processedData;
    }),
    catchError((err) => {
      console.error('Erreur API :', err);
      throw new Error('La requête API a échoué');
    })
  );
}

// Surveiller les modifications des critères de recherche
const searchTerm$ = fromEvent(searchInput, 'input').pipe(
  map((event) => (event.target as HTMLInputElement).value.trim()),
  debounceTime(300),
  distinctUntilChanged()
);

// Surveiller les modifications des paramètres du cache
const useCache$ = fromEvent(cacheCheckbox, 'change').pipe(
  map((event) => (event.target as HTMLInputElement).checked),
  startWith(true)
);

// Surveiller les modifications pour forcer le rechargement
const forceRefresh$ = fromEvent(forceCheckbox, 'change').pipe(
  map((event) => (event.target as HTMLInputElement).checked),
  startWith(false)
);

// Événement de clic sur le bouton de recherche
const searchClick$ = fromEvent(searchButton, 'click');

// Exécution de la recherche
searchClick$
  .pipe(
    // Récupérer les valeurs d'entrée actuelles, les paramètres du cache et les paramètres de rechargement forcé
    withLatestFrom(
      searchTerm$,
      useCache$,
      forceRefresh$,
      (_, term, useCache, forceRefresh) => ({
        term,
        useCache,
        forceRefresh,
      })
    ),
    tap(() => {
      // Affichage du début de la recherche
      optimizedResults.innerHTML = '<p>Recherche en cours...</p>';
    }),
    // Flux conditionnel utilisant iif()
    switchMap(({ term, useCache, forceRefresh }) => {
      // Si le terme de recherche est vide
      if (!term) {
        return of({ error: 'Veuillez entrer un terme de recherche' });
      }

      // Si le cache est désactivé
      if (!useCache) {
        return fetchUserData(term, true);
      }

      // Recherche normale (utiliser le cache et forcer le rechargement si nécessaire)
      return fetchUserData(term, forceRefresh);
    }),
    // Gestion des erreurs
    catchError((err) => {
      return of({ error: err.message });
    })
  )
  .subscribe({
    next: (result) => {
      if ('error' in result) {
        // Affichage des erreurs
        optimizedResults.innerHTML = `
        <p style="color: red;">Erreur : ${result.error}</p>
      `;
      } else {
        // Affichage des données
        const source = result.fromCache
          ? '<span style="color: orange;">(depuis le cache)</span>'
          : '<span style="color: green;">(depuis l\'API)</span>';

        optimizedResults.innerHTML = `
        <h4>Informations utilisateur ${source}</h4>
        <p><strong>ID :</strong> ${result.id}</p>
        <p><strong>Nom :</strong> ${result.name}</p>
        <p><strong>Email :</strong> ${result.email}</p>
        ${
          result.lastUpdated
            ? `<p><small>Dernière mise à jour : ${new Date(
                result.lastUpdated
              ).toLocaleString()}</small></p>`
            : ''
        }
      `;
      }
    },
  });

// Message initial
optimizedResults.innerHTML =
  '<p>Entrez l\'ID utilisateur et cliquez sur le bouton de recherche</p>';

```


---

## Guide de sélection des opérateurs

De nombreux opérateurs conditionnels se ressemblent et peuvent prêter à confusion, mais chacun d'entre eux a un objectif d'application clair.
Vous trouverez ci-dessous une comparaison des flux de décision et des caractéristiques typiques.

## Comment choisir un opérateur conditionnel

| Opérateur | Cas d'utilisation | Caractéristiques |
|------------|------------|------|
| `iif` | Sélectionner un flux au moment de l'exécution | Sélectionner l'une des deux alternatives basées sur une condition |
| `partition` | Diviser le flux en deux par condition | Diviser le flux d'origine en Vrai/Faux par condition |
| `throwIfEmpty` | Détecter les flux vides | Lancer une erreur si aucune valeur n'est émise |
| `defaultIfEmpty` | Utiliser la valeur par défaut si vide | Fournir une valeur de repli si le flux est vide |

### Flux de décision de sélection

1. **Y a-t-il deux choix ?**
   - Oui → Utiliser `iif`
   - Non → Suivant

2. **Voulez-vous diviser le flux ?**
   - Oui → Utiliser `partition`
   - Non → Suivant

3. **Voulez-vous traiter les flux vides ?**
   - Oui → Voulez-vous traiter les flux vides comme des erreurs ?
     - Oui → `throwIfEmpty`
     - Non → `defaultIfEmpty`
   - Non → Suivant

4. **Voulez-vous simplement filtrer des valeurs en fonction d'une condition ?**
   - Oui → Utiliser l'opérateur `filter` (opérateur de filtrage de base)
   - Non → Reconsidérer l'objectif

## Résumé

Les opérateurs conditionnels sont des outils puissants qui permettent de contrôler le flux de données et d'effectuer des traitements de branchement en fonction de conditions spécifiques. Les points clés sont les suivants :

1. **Flux réactif basé sur la décision** : les opérateurs conditionnels vous permettent de modifier dynamiquement le traitement en fonction d'événements ou de conditions de données.
2. **Gestion améliorée des erreurs** : les opérateurs conditionnels jouent un rôle important dans la stratégie de gestion des erreurs et permettent de traiter les cas d'exception avec élégance.
3. **Optimisation** : l'exécution conditionnelle permet d'éviter les traitements inutiles et d'optimiser les opérations coûteuses, en particulier les requêtes réseau et l'accès au matériel.
4. **Flux d'applications complexes** : en combinant plusieurs opérateurs conditionnels, il est possible d'exprimer de manière déclarative une logique métier et une gestion d'état complexes.

Les opérateurs conditionnels sont particulièrement utiles lors de la mise en œuvre de la gestion des erreurs, des stratégies de mise en cache, des mécanismes de repli et des modèles d'exécution conditionnelle à l'aide de RxJS. En combinaison avec d'autres opérateurs, des flux d'application complexes peuvent être construits de manière déclarative et type-safe.
