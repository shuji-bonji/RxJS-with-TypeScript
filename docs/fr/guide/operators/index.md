---
description: "Les opérateurs RxJS sont classés en sept groupes : transformation, filtrage, combinaison, utilitaire, conditionnel, traitement des erreurs et multidiffusion. Apprenez l'utilisation pratique de TypeScript avec des listes complètes d'opérateurs et des concepts de pipeline."
---

# Comprendre les opérateurs

Les opérateurs RxJS sont un ensemble de fonctions pour transformer, composer et contrôler les flux de données Observable.

Les opérateurs sont généralement utilisés en combinaison avec plusieurs autres, et c'est là qu'intervient le "pipeline".
- [Qu'est-ce que le pipeline RxJS](./pipeline.md)

Dans RxJS, les opérateurs se répartissent dans les catégories suivantes


## Liste des catégories

- [Opérateurs de transformation](./transformation/)
- [Opérateurs de filtrage](./filtering/)
- [Opérateurs de combinaison](./combination/)
- [Opérateurs utilitaires](./utility/)
- [Opérateurs conditionnels](./conditional/)
- [Opérateurs de gestion des erreurs](../error-handling/strategies.md)
- [Opérateurs de multidiffusion](./multicasting/)

Chaque catégorie contient un certain nombre d'opérateurs utiles.
Voir chaque catégorie pour plus de détails.


## Liste des opérateurs

Pour une description détaillée de chaque opérateur, cliquez sur le lien pour parcourir.

<table style="overflow: visible;">
  <caption>
   Liste des catégories d'opérateurs
  </caption>
  <thead>
    <tr>
      <th scope="col">Catégorie</th>
      <th scope="col">Opérateur</th>
      <th scope="col">Description</th>
    </tr>
  </thead>
  <tbody>
    <!-- Opérateurs de transformation -->
    <tr>
      <th scope="row" rowspan="15"><a href="./transformation/">Transformation</a></th>
      <td><a href="./transformation/map.html">map</a></td>
      <td>Convertit chaque valeur</td>
    </tr>
    <tr>
      <td><a href="./transformation/scan.html">scan</a></td>
      <td>Accumule les valeurs et produit des résultats intermédiaires</td>
    </tr>
    <tr>
      <td><a href="./transformation/reduce.html">reduce</a></td>
      <td>Accumule toutes les valeurs et n'émet que le résultat final</td>
    </tr>
    <tr>
      <td><a href="./transformation/pairwise.html">pairwise</a></td>
      <td>Traite deux valeurs consécutives par paires</td>
    </tr>
    <tr>
      <td><a href="./transformation/groupBy.html">groupBy</a></td>
      <td>Regroupement des flux par clé</td>
    </tr>
    <tr>
      <td><a href="./transformation/mergeMap.html">mergeMap</a></td>
      <td>Exécution parallèle du traitement asynchrone</td>
    </tr>
    <tr>
      <td><a href="./transformation/switchMap.html">switchMap</a></td>
      <td>Exécuter uniquement le dernier traitement asynchrone (annuler les traitements plus anciens)</td>
    </tr>
    <tr>
      <td><a href="./transformation/concatMap.html">concatMap</a></td>
      <td>Exécution séquentielle des processus asynchrones</td>
    </tr>
    <tr>
      <td><a href="./transformation/exhaustMap.html">exhaustMap</a></td>
      <td>Ignorer les nouveaux processus pendant l'exécution</td>
    </tr>
    <tr>
      <td><a href="./transformation/expand.html">expand</a></td>
      <td>Développer les résultats de manière récursive</td>
    </tr>
    <tr>
      <td><a href="./transformation/buffer.html">buffer</a></td>
      <td>Publier des valeurs dans un tableau</td>
    </tr>
    <tr>
      <td><a href="./transformation/bufferTime.html">bufferTime</a></td>
      <td>Publier des valeurs à des intervalles de temps spécifiés</td>
    </tr>
    <tr>
      <td><a href="./transformation/bufferCount.html">bufferCount</a></td>
      <td>Publier des valeurs par lots d'un nombre spécifié de valeurs</td>
    </tr>
    <tr>
      <td><a href="./transformation/bufferWhen.html">bufferWhen</a></td>
      <td>Mise en mémoire tampon avec conditions de fin contrôlées dynamiquement</td>
    </tr>
    <tr>
      <td><a href="./transformation/bufferToggle.html">bufferToggle</a></td>
      <td>Mise en mémoire tampon avec contrôle indépendant du début et de la fin</td>
    </tr>
    <!-- Opérateurs de filtrage -->
    <tr>
      <th scope="row" rowspan="22"><a href="./filtering/">Filtrage</a></th>
      <td><a href="./filtering/filter.html">filter</a></td>
      <td>Ne laisser passer que les valeurs qui correspondent à la condition</td>
    </tr>
    <tr>
      <td><a href="./filtering/take.html">take</a></td>
      <td>N'obtenir que les N premières valeurs</td>
    </tr>
    <tr>
      <td><a href="./filtering/takeLast.html">takeLast</a></td>
      <td>Obtenir les N dernières valeurs</td>
    </tr>
    <tr>
      <td><a href="./filtering/takeWhile.html">takeWhile</a></td>
      <td>Obtenir les valeurs lorsque la condition est remplie</td>
    </tr>
    <tr>
      <td><a href="./filtering/skip.html">skip</a></td>
      <td>Sauter les N premières valeurs</td>
    </tr>
    <tr>
      <td><a href="./filtering/skipLast.html">skipLast</a></td>
      <td>Sauter les N dernières valeurs</td>
    </tr>
    <tr>
      <td><a href="./filtering/skipWhile.html">skipWhile</a></td>
      <td>Sauter les valeurs pendant que la condition est remplie</td>
    </tr>
    <tr>
      <td><a href="./filtering/skipUntil.html">skipUntil</a></td>
      <td>Sauter les valeurs jusqu'à ce qu'un autre Observable se déclenche</td>
    </tr>
    <tr>
      <td><a href="./filtering/first.html">first</a></td>
      <td>Obtenir la première valeur ou la première valeur satisfaisant une condition</td>
    </tr>
    <tr>
      <td><a href="./filtering/last.html">last</a></td>
      <td>Obtenir la dernière valeur ou la dernière valeur satisfaisant la condition</td>
    </tr>
    <tr>
      <td><a href="./filtering/elementAt.html">elementAt</a></td>
      <td>Obtenir la valeur à un index donné</td>
    </tr>
    <tr>
      <td><a href="./filtering/find.html">find</a></td>
      <td>Trouver la première valeur qui remplit une condition</td>
    </tr>
    <tr>
      <td><a href="./filtering/findIndex.html">findIndex</a></td>
      <td>Obtenir l'indice de la première valeur qui satisfait la condition</td>
    </tr>
    <tr>
      <td><a href="./filtering/debounceTime.html">debounceTime</a></td>
      <td>Émettre la dernière valeur si aucune entrée n'est reçue pendant un certain temps</td>
    </tr>
    <tr>
      <td><a href="./filtering/throttleTime.html">throttleTime</a></td>
      <td>Passer la première valeur et ignorer la nouvelle valeur pendant le temps spécifié</td>
    </tr>
    <tr>
      <td><a href="./filtering/auditTime.html">auditTime</a></td>
      <td>Délivrer la dernière valeur après le délai spécifié</td>
    </tr>
    <tr>
      <td><a href="./filtering/audit.html">audit</a></td>
      <td>Émettre la dernière valeur avec un Observable personnalisé pour contrôler la période</td>
    </tr>
    <tr>
      <td><a href="./filtering/sampleTime.html">sampleTime</a></td>
      <td>Échantillonner la dernière valeur à l'intervalle de temps spécifié</td>
    </tr>
    <tr>
      <td><a href="./filtering/ignoreElements.html">ignoreElements</a></td>
      <td>Ignorer toutes les valeurs et ne transmettre que les achèvements/erreurs</td>
    </tr>
    <tr>
      <td><a href="./filtering/distinct.html">distinct</a></td>
      <td>Supprimer toutes les valeurs en double (ne transmettre que les valeurs uniques)</td>
    </tr>
    <tr>
      <td><a href="./filtering/distinctUntilChanged.html">distinctUntilChanged</a></td>
      <td>Supprimer les valeurs dupliquées consécutives</td>
    </tr>
    <tr>
      <td><a href="./filtering/distinctUntilKeyChanged.html">distinctUntilKeyChanged</a></td>
      <td>Détecter uniquement les changements de propriétés spécifiques d'un objet</td>
    </tr>
    <!-- Opérateurs de combinaison (Pipeable) -->
    <tr>
      <th scope="row" rowspan="12"><a href="./combination/">Combinaison (Pipeable)</a></th>
      <td><a href="./combination/concatWith.html">concatWith</a></td>
      <td>Rejoindre d'autres Observables dans la séquence après achèvement</td>
    </tr>
    <tr>
      <td><a href="./combination/mergeWith.html">mergeWith</a></td>
      <td>Combiner plusieurs Observables simultanément</td>
    </tr>
    <tr>
      <td><a href="./combination/combineLatestWith.html">combineLatestWith</a></td>
      <td>Combiner la dernière valeur de chaque Observable</td>
    </tr>
    <tr>
      <td><a href="./combination/zipWith.html">zipWith</a></td>
      <td>Associer les valeurs dans l'ordre correspondant</td>
    </tr>
    <tr>
      <td><a href="./combination/raceWith.html">raceWith</a></td>
      <td>Adopter uniquement le premier Observable qui se déclenche</td>
    </tr>
    <tr>
      <td><a href="./combination/withLatestFrom.html">withLatestFrom</a></td>
      <td>Ajouter les autres valeurs les plus récentes au flux principal</td>
    </tr>
    <tr>
      <td><a href="./combination/mergeAll.html">mergeAll</a></td>
      <td>Aplatir les Observables d'ordre supérieur en parallèle</td>
    </tr>
    <tr>
      <td><a href="./combination/concatAll.html">concatAll</a></td>
      <td>Aplatir les Observables d'ordre supérieur dans l'ordre</td>
    </tr>
    <tr>
      <td><a href="./combination/switchAll.html">switchAll</a></td>
      <td>Passer à l'Observable d'ordre supérieur le plus récent</td>
    </tr>
    <tr>
      <td><a href="./combination/exhaustAll.html">exhaustAll</a></td>
      <td>Ignorer le nouvel Observable d'ordre supérieur pendant l'exécution</td>
    </tr>
    <tr>
      <td><a href="./combination/combineLatestAll.html">combineLatestAll</a></td>
      <td>Combine les dernières valeurs de tous les Observables internes</td>
    </tr>
    <tr>
      <td><a href="./combination/zipAll.html">zipAll</a></td>
      <td>Associe les valeurs correspondantes de chaque Observable interne</td>
    </tr>
    <!-- Opérateurs utilitaires -->
    <tr>
      <th scope="row" rowspan="15"><a href="./utility/">Utilitaire</a></th>
      <td><a href="./utility/tap.html">tap</a></td>
      <td>Effectuer des effets de bord (par exemple, sortie d'un journal)</td>
    </tr>
    <tr>
      <td><a href="./utility/finalize.html">finalize</a></td>
      <td>Effectuer un post-traitement en cas d'achèvement ou d'erreur</td>
    </tr>
    <tr>
      <td><a href="./utility/delay.html">delay</a></td>
      <td>Retarder toutes les valeurs pendant une durée déterminée</td>
    </tr>
    <tr>
      <td><a href="./utility/delayWhen.html">delayWhen</a></td>
      <td>Retarder chaque valeur dynamiquement avec un Observable séparé</td>
    </tr>
    <tr>
      <td><a href="./utility/timeout.html">timeout</a></td>
      <td>Émettre une erreur si une valeur n'arrive pas dans un délai spécifié</td>
    </tr>
    <tr>
      <td><a href="./utility/takeUntil.html">takeUntil</a></td>
      <td>Récupérer les valeurs jusqu'à ce qu'un autre Observable émette une valeur</td>
    </tr>
    <tr>
      <td><a href="./utility/retry.html">retry</a></td>
      <td>Réessayer jusqu'à un nombre spécifié de fois en cas d'erreur</td>
    </tr>
    <tr>
      <td><a href="./utility/repeat.html">repeat</a></td>
      <td>Répéter un nombre spécifié de fois après achèvement</td>
    </tr>
    <tr>
      <td><a href="./utility/startWith.html">startWith</a></td>
      <td>Ajoute une valeur initiale au début du flux</td>
    </tr>
    <tr>
      <td><a href="./utility/toArray.html">toArray</a></td>
      <td>Publier toutes les valeurs dans un tableau</td>
    </tr>
    <tr>
      <td><a href="./utility/materialize.html">materialize</a></td>
      <td>Convertit une notification en un objet Notification</td>
    </tr>
    <tr>
      <td><a href="./utility/dematerialize.html">dematerialize</a></td>
      <td>Reconvertir l'objet Notification en notification normale</td>
    </tr>
    <tr>
      <td><a href="./utility/observeOn.html">observeOn</a></td>
      <td>Utiliser le scheduler pour contrôler le moment où les valeurs sont publiées</td>
    </tr>
    <tr>
      <td><a href="./utility/subscribeOn.html">subscribeOn</a></td>
      <td>Utiliser le scheduler pour contrôler le moment où l'on commence à s'abonner</td>
    </tr>
    <tr>
      <td><a href="./utility/timestamp.html">timestamp</a></td>
      <td>Ajouter un horodatage à chaque valeur</td>
    </tr>
    <!-- Opérateurs conditionnels -->
    <tr>
      <th scope="row" rowspan="3"><a href="./conditional/">Conditionnel</a></th>
      <td><a href="./conditional/defaultIfEmpty.html">defaultIfEmpty</a></td>
      <td>Si aucune valeur n'est disponible, émettre une valeur par défaut</td>
    </tr>
    <tr>
      <td><a href="./conditional/every.html">every</a></td>
      <td>Déterminer si toutes les valeurs satisfont à la condition</td>
    </tr>
    <tr>
      <td><a href="./conditional/isEmpty.html">isEmpty</a></td>
      <td>Détermine si aucune valeur n'a été émise</td>
    </tr>
    <!-- Gestion des erreurs -->
    <tr>
      <th scope="row" rowspan="3"><a href="../error-handling/strategies.html">Gestion des erreurs</a></th>
      <td><a href="../error-handling/retry-catch.html">catchError</a></td>
      <td>Attraper les erreurs et effectuer un traitement de secours</td>
    </tr>
    <tr>
      <td><a href="../error-handling/retry-catch.html">retry</a></td>
      <td>Réessayer un certain nombre de fois en cas d'erreur</td>
    </tr>
    <tr>
      <td><a href="../error-handling/retry-catch.html">retryWhen</a></td>
      <td>Réessayer avec des conditions personnalisées</td>
    </tr>
    <!-- Multidiffusion -->
    <tr>
      <th scope="row" rowspan="2"><a href="./multicasting/">Multidiffusion</a></th>
      <td><a href="./multicasting/share.html">share</a></td>
      <td>Partage de l'Observable entre plusieurs abonnés</td>
    </tr>
    <tr>
      <td><a href="./multicasting/shareReplay.html">shareReplay</a></td>
      <td>Mettre en cache les N dernières valeurs et les rediffuser aux nouveaux abonnés</td>
    </tr>
  </tbody>
</table>
