#[Handbook of Robotics Second ed](http://www.springer.com/us/book/9783319325507)
###56
The introduction in this chapter covers most of the important motivations for robotics in agriculture. First is defines ag.
>In this chapter, agriculture is understood as in the Merriam-Webster definition:
the science, art, or practice of cultivating the soil, producing crops, and raising livestock and in vary- ing degrees the preparation and marketing of the resulting products.

It goes on to define the term crop.
>the term crop may be used here to denote any product of an agricultural or forestry process, includ- ing grains, cereals, fruit, vegetables, nuts, trees, beef, wool, etc.

This is a very important disinction as many people thing of just plants as a "crop".  

Also the famous line for the United Nations is used.
>Scientists predict that agricultural production must double to meet the demands of nine billion people in 2050 [56.1–3].

###56.2
This subsection covers the chalanges that robotics is facing as it pertains to agriculture. One of the challenges is auto guidance on the field. This is defined as the abilty of a machine to manuver a field with out damage to the crops. The imporance of autonavition is mentioned here.
>More recently, auto-guidance has started to migrate to orchard vehicles as well, albeit here other navigation sensors may be required because of poor satellite reception under thick canopies

This defends the importance to navigation means that are not fixed to GNSS. Also the author talks of the big problems with auto navigation sensors. Saying
>In general, robotic mobility technology is currently less advanced than sensing.

Sensing here talks about the crops state not the robots localization.

###56.3.2
This was the case study of a automated weeding system for potatos. The important fact was how it handled weed vs plant. Normaly this is difficult with computer vision as they are around the same color. The solution was to use a texture based system.
>Because weeds and grass are both green, a texture-based image analysis method was used to de- tect the former [56.22,23].


##Important refs
- [56.1] J.A. Foley, N. Ramankutty, K.A. Brauman, E.S. Cas- sidy, J.S. Gerber, M. Johnston, N.D. Mueller, C. O’Connell, D.K. Ray, P.C. West, C. Balzer, E.M. Ben- nett, S.R. Carpenter, J. Hill, C. Monfreda, S. Polasky, J. Rockström, J. Sheehan, S. Siebert, D. Tilman, D.P.M. Zaks: Solutions for a cultivated planet, Na- ture 478, 337–342 (2011)  
- [56.2] A. Alleyne: Editor’s note: Agriculture and infor- mation technology, Natl. Acad. Eng. Bridge, Issue Agric. Inf, Technol. 41(3), 3–4 (2011)  
- [56.3] Global Harvest Initiative: The 2012 Global Agricul- tural Productivity Report, http://goo.gl/GBPvjK
- [56.22] G. Polder, F.K. Van Evert, A. Lamaker, A. De Jong, G.W.A.M. Van der Heijden, L.A.P. Lotz, T. Van der Zalm, C. Kempenaar: Weed detection using textural image analysis, 6th Bienn. Conf. Eur. Fed. IT Agric. (EFITA), Glasgow (2007), available online at http:// edepot.wur.nl/28203
- [56.23] F.K. Van Evert, G. Polder, G.W.A.M. Van der Heij- den, C. Kempenaar, L.A.P. Lotz: Real-time, vision- based detection of Rumex obtusifolius L. in grass- land, Weed Res. 49, 164–174 (2009)

---

